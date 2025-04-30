from typing import Optional
import numpy as np

import rclpy
from rclpy.publisher import Publisher
from rclpy.time import Time
from rclpy.timer import Timer
from rclpy.duration import Duration
import time
from navigation import approach_target, stuck_recovery, waypoint, state
from navigation.astar import AStar, SpiralEnd, NoPath, OutOfBounds
from navigation.coordinate_utils import d_calc, gen_marker, is_high_cost_point, cartesian_to_ij
from navigation.context import Context
from navigation.trajectory import Trajectory, SearchTrajectory
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from state_machine.state import State
from rclpy.publisher import Publisher


# REFERENCE: https://docs.google.com/document/d/18GjDWxIu5f5-N5t5UgbrZGdEyaDj9ZMEUuXex8-NKrA/edit
class CostmapSearchState(State):
    """
    General search state that utilizes the costmap for searching for any obstacles
    Follows a search spiral but uses A* to avoid obstacles
    Intended to be implemented over the current search state
    """

    time_begin: Time
    marker_timer: Timer
    update_astar_timer: Timer | None

    spiral_traj: SearchTrajectory
    astar_traj: Trajectory  # returned by astar
    prev_target_pos_in_map: Optional[np.ndarray] = None
    is_recovering: bool = False
    path_pub: Publisher
    astar: AStar
    marker_pub: Publisher

    USE_COSTMAP: bool
    STOP_THRESH: float
    DRIVE_FWD_THRESH: float
    UPDATE_DELAY: float

    def on_enter(self, context: Context) -> None:
        context.node.get_logger().info("Entered Costmap Search State")
        context.rover.previous_state = CostmapSearchState()

        if context.course is None:
            return

        current_waypoint = context.course.current_waypoint()
        if current_waypoint is None:
            return

        self.USE_COSTMAP = context.node.get_parameter("costmap.use_costmap").value or current_waypoint.enable_costmap

        self.STOP_THRESH = context.node.get_parameter("search.stop_threshold").value
        self.DRIVE_FWD_THRESH = context.node.get_parameter("search.drive_forward_threshold").value
        self.UPDATE_DELAY = context.node.get_parameter("search.update_delay").value

        self.time_begin = context.node.get_clock().now()
        self.marker_pub = context.node.create_publisher(Marker, "spiral_points", 10)

        self.new_traj(context)

        if not self.is_recovering:
            self.prev_target_pos_in_map = None

        self.astar = AStar(context)
        self.astar_traj = Trajectory(np.array([]))
        self.time_begin = context.node.get_clock().now()

        self.marker_timer = context.node.create_timer(
            context.node.get_parameter("pub_path_rate").value, lambda: self.display_markers(context)
        )
        self.update_astar_timer = None

    def on_exit(self, context: Context) -> None:
        self.marker_timer.cancel()
        if self.update_astar_timer is not None:
            self.update_astar_timer.cancel()
        self.marker_pub.publish(gen_marker(context, delete=True))

    def display_markers(self, context: Context) -> None:
        start_pt = self.spiral_traj.cur_pt
        end_pt = (
            self.spiral_traj.cur_pt + 6
            if self.spiral_traj.cur_pt + 3 < len(self.spiral_traj.coordinates)
            else len(self.spiral_traj.coordinates)
        )
        if context.node.get_parameter("display_markers").value:
            for i, coord in enumerate(self.spiral_traj.coordinates[start_pt:end_pt]):
                self.marker_pub.publish(
                    gen_marker(
                        context=context,
                        point=coord,
                        color=[1.0, 0.0, 0.0],
                        id=i,
                        lifetime=context.node.get_parameter("pub_path_rate").value,
                    )
                )

    def update_astar_traj(self, context: Context):
        if context.course is None:
            return
        context.rover.send_drive_command(Twist())
        try:
            self.astar_traj = self.astar.generate_trajectory(context, self.spiral_traj.get_current_point())
        except Exception as e:
                context.node.get_logger().info(str(e))
                return self
        # If a spiral point is unreachable, then skip it and generate a new trajectory
        if self.astar_traj.empty():
            context.node.get_logger().info(f"Skipping unreachable spiral point")
            self.spiral_traj.increment_point()
            context.course.last_spiral_point = self.spiral_traj.cur_pt
            if self.spiral_traj.done():
                context.node.get_logger().info(f"Reached end of search spiral")
                return
            return

    def on_loop_costmap_enabled(self, context: Context) -> State:
        if not self.USE_COSTMAP:
            return self
        
        if not context.dilation_done():
            context.node.get_logger().info("Awaiting dilation future to complete")
            return self

        if self.update_astar_timer is None:
            self.update_astar_timer = context.node.create_timer(
                self.UPDATE_DELAY, lambda: self.update_astar_traj(context)
            )
            return self

        if context.env.cost_map is None or not hasattr(context.env.cost_map, "data"):
            context.node.get_logger().warn("Costmap is enabled but costmap has no data")
            return self

        rover_pose = context.rover.get_pose_in_map()
        if rover_pose is None:
            context.node.get_logger().warn("Rover has no pose, waiting...")
            context.rover.send_drive_command(Twist())
            return self

        costmap_length = context.env.cost_map.data.shape[0]

        # Skip spiral points until we find one that is not high cost
        while is_high_cost_point(context=context, point=self.spiral_traj.get_current_point()):
            context.node.get_logger().info(f"Skipping high cost spiral point")
            self.spiral_traj.increment_point()
            if context.course is None:
                return self
            context.course.last_spiral_point = self.spiral_traj.cur_pt

            # If we reach the end of the spiral trajectory via skipping high cost points, go back into the waypoint state
            if self.spiral_traj.done():
                self.spiral_traj.clear()
                context.node.get_logger().info(f"Reached end of search spiral")
                return waypoint.WaypointState()

            spiral_point_ij = cartesian_to_ij(context, self.spiral_traj.get_current_point())
            # If we skipped to a point outside the costmap, begin the spiral again
            # TODO: Determine if this is desired behavior
            if not (0 <= int(spiral_point_ij[0]) < costmap_length and 0 <= int(spiral_point_ij[1]) < costmap_length):
                self.spiral_traj.reset()
                break

            self.astar_traj.clear()

        curr_point = cartesian_to_ij(context, self.spiral_traj.get_current_point())
        if not 0 <= int(curr_point[0]) < costmap_length and 0 <= int(curr_point[1]) < costmap_length:
            context.node.get_logger().warn(
                f"Cart Point: {self.spiral_traj.get_current_point()} IJ Point: {curr_point} and cost: {context.env.cost_map.data[curr_point[0]][curr_point[1]]} \n Trajectory point out of the map. Going back to last point in costmap and trying again..."
            )
            dest_ij = cartesian_to_ij(context, self.spiral_traj.get_current_point())
            while not (0 <= int(dest_ij[0]) < costmap_length and 0 <= int(dest_ij[1]) < costmap_length):
                # a lil boof ngl
                if self.spiral_traj.decerement_point():
                    break
                dest_ij = cartesian_to_ij(context, self.spiral_traj.get_current_point())

            return self

        # If there are no more points in the current a_star path, then create a new one
        if self.astar_traj.empty():
            self.update_astar_traj(context)
            if self.spiral_traj.done():
                self.spiral_traj.clear()
                context.node.get_logger().info(f"Reached end of search spiral")
                return waypoint.WaypointState()
            return self

        target_position_in_map = self.astar_traj.get_current_point()

        cmd_vel, arrived = context.drive.get_drive_command(
            target_position_in_map,
            context.rover.get_pose_in_map(),
            self.STOP_THRESH,
            self.DRIVE_FWD_THRESH,
            path_start=self.prev_target_pos_in_map,
        )

        if not arrived:
            context.rover.send_drive_command(cmd_vel)
        else:
            self.prev_target_pos_in_map = target_position_in_map
            if self.astar_traj.increment_point():
                self.astar_traj.clear()
                self.spiral_traj.increment_point()
                if context.course is None:
                    return self
                context.course.last_spiral_point = self.spiral_traj.cur_pt
                if self.spiral_traj.done():
                    context.node.get_logger().info(f"Reached end of search spiral")
                    return waypoint.WaypointState()

        return self

    def on_loop_costmap_disabled(self, context: Context):
        if self.USE_COSTMAP:
            return self
    
        if context.course is None:
            return self 
        
        curr_spiral_point = self.spiral_traj.get_current_point()
        cmd_vel, arrived = context.drive.get_drive_command(
            curr_spiral_point,
            context.rover.get_pose_in_map(),
            context.node.get_parameter("search.stop_threshold").value,
            context.node.get_parameter("search.drive_forward_threshold").value,
            path_start=self.prev_target_pos_in_map,
        )
        if arrived:
            self.prev_target_pos_in_map = curr_spiral_point
            # If we finish the spiral without seeing the tag, return back to waypoint state to try again
            self.spiral_traj.increment_point()
            context.course.last_spiral_point = self.spiral_traj.cur_pt
            if self.spiral_traj.done():
                return waypoint.WaypointState()

        if context.rover.stuck:
            context.rover.previous_state = self
            self.is_recovering = True
            return stuck_recovery.StuckRecoveryState()
        else:
            self.is_recovering = False

        context.rover.send_drive_command(cmd_vel)
        return self

    def on_loop(self, context: Context) -> State:
        # Wait until the costmap is ready
        if not hasattr(context.env.cost_map, "data") and context.node.get_parameter("costmap.use_costmap").value:
            context.node.get_logger().warn(f"No costmap found, waiting...")
            return self

        if context.course is None:
            return state.DoneState()

        rover_in_map = context.rover.get_pose_in_map()
        if rover_in_map is None:
            context.node.get_logger().warn("Rover has no pose, waiting...")
            context.rover.send_drive_command(Twist())
            return self

        if context.rover.stuck:
            context.rover.previous_state = self
            self.is_recovering = True
            return stuck_recovery.StuckRecoveryState()
        else:
            self.is_recovering = False

        if context.node.get_clock().now() < self.time_begin + Duration(seconds=self.UPDATE_DELAY // 2):
            return self

        # Check if we belong in any other state
        approach_state = context.course.get_approach_state()
        if approach_state is not None:
            total_time = context.node.get_clock().now() - self.time_begin
            context.node.get_logger().info(f"Total search time: {total_time.nanoseconds // 1000000000}")
            return approach_state

        if self.USE_COSTMAP:
            return self.on_loop_costmap_enabled(context=context)
        else:
            return self.on_loop_costmap_disabled(context=context)

    def new_traj(self, context: Context) -> None:
        if context.course is None:
            return
        
        search_center = context.course.current_waypoint()
        if search_center is None:
            return

        if not self.is_recovering:
            self.spiral_traj = SearchTrajectory.spiral_traj(
                center=context.course.current_waypoint_pose_in_map().translation()[0:2],
                coverage_radius=context.node.get_parameter("search.coverage_radius").value,
                distance_between_spirals=context.node.get_parameter("search.distance_between_spirals").value,
                segments_per_rotation=context.node.get_parameter("search.segments_per_rotation").value,
                max_segment_length=context.node.get_parameter("search.max_segment_length").value,
                tag_id=search_center.tag_id,
                insert_extra=True,
            )
        
        self.spiral_traj.cur_pt = context.course.last_spiral_point
