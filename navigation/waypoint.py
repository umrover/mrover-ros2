from state_machine.state import State
from . import (
    backup,
    state,
    costmap_search,
    stuck_recovery,
)
from mrover.msg import WaypointType
from mrover.srv import MoveCostMap
from .context import Context
import rclpy
from .context import Context
from navigation.astar import AStar, SpiralEnd, NoPath, OutOfBounds, DestinationInHighCost
from navigation.coordinate_utils import segment_path, is_high_cost_point, d_calc, cartesian_to_ij
from navigation.marker_utils import gen_marker
from navigation.trajectory import Trajectory, SearchTrajectory
from typing import Optional
from rclpy.publisher import Publisher
from rclpy.time import Time
from rclpy.timer import Timer
import time
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import numpy as np


class WaypointState(State):
    # STOP_THRESHOLD: float = rospy.get_param("waypoint/stop_threshold")
    # DRIVE_FORWARD_THRESHOLD: float = rospy.get_param("waypoint/drive_forward_threshold")
    # USE_COSTMAP: bool = rospy.get_param("water_bottle_search.use_costmap")
    # NO_TAG: int = -1
    astar_traj: Trajectory
    waypoint_traj: Trajectory
    prev_target_pos_in_map: Optional[np.ndarray] = None
    is_recovering: bool = False
    time_no_search_wait: Optional[Time] = None
    time_begin: Time
    start_time: Time
    marker_timer: Timer
    waypoint_timer: Timer
    path_pub: Publisher
    astar: AStar
    marker_pub: Publisher

    UPDATE_DELAY: float
    NO_SEARCH_WAIT_TIME: float
    USE_COSTMAP: bool
    USE_PURE_PURSUIT: bool

    def on_enter(self, context: Context) -> None:
        if context.course is None:
            return

        self.time_begin = context.node.get_clock().now()
        self.start_time = context.node.get_clock().now()

        context.node.get_logger().info("Entered Waypoint State")
        context.dilate_cost(1.0)
        context.rover.previous_state = WaypointState()

        context.course.last_spiral_point = 0

        self.UPDATE_DELAY = context.node.get_parameter("search.update_delay").value
        self.NO_SEARCH_WAIT_TIME = context.node.get_parameter("waypoint.no_search_wait_time").value

        self.marker_pub = context.node.create_publisher(Marker, "waypoint_trajectory", 10)
        self.astar = AStar(context)
        self.astar_traj = Trajectory(np.array([]))
        self.waypoint_traj = Trajectory(np.array([]))

        self.time_no_search_wait = None

        self.marker_timer = context.node.create_timer(
            context.node.get_parameter("pub_path_rate").value, lambda: self.display_markers(context)
        )
        self.waypoint_timer = context.node.create_timer(self.UPDATE_DELAY, lambda: self.update_waypoint(context))

        current_waypoint = context.course.current_waypoint()
        if current_waypoint is None:
            return

        self.USE_PURE_PURSUIT = context.node.get_parameter("pure_pursuit.use_pure_pursuit").value
        self.USE_COSTMAP = context.node.get_parameter_or("costmap.use_costmap", True).value or current_waypoint.enable_costmap
        if self.USE_COSTMAP:
            context.node.get_logger().info("Resetting costmap dilation")
            context.reset_dilation()

        context.node.get_logger().info("On Enter finished")

    def on_exit(self, context: Context) -> None:
        self.marker_timer.cancel()
        self.waypoint_timer.cancel()

    def update_waypoint(self, context: Context) -> None:
        self.waypoint_traj.clear()
        self.astar_traj.clear()

    def on_loop_costmap_enabled(self, context: Context) -> State:
        if not context.dilation_done():
            context.node.get_logger().info("Awaiting dilation future to complete")
            return self

        if not self.USE_COSTMAP:
            return self
        if context.course is None:
            return state.DoneState()

        # charlie beck helped me debug this
        if not hasattr(context.env.cost_map, "data") and self.USE_COSTMAP:
            context.node.get_logger().warn(f"No costmap found, waiting...")
            self.start_time = context.node.get_clock().now()
            return self

        rover_pose = context.rover.get_pose_in_map()
        if rover_pose is None:
            context.node.get_logger().warn("Rover has no pose, waiting...")
            context.rover.send_drive_command(Twist())
            return self

        if self.waypoint_traj.empty():
            context.node.get_logger().info("Generating segmented path")
            self.waypoint_traj = segment_path(
                context=context, dest=context.course.current_waypoint_pose_in_map().translation()[0:2]
            )
            self.display_markers(context=context)
            return self

        # BEGINNING OF LOGIC
        while (
            is_high_cost_point(context=context, point=self.waypoint_traj.get_current_point())
            and not self.waypoint_traj.is_last()
        ):
            context.node.get_logger().info("Skipping high cost point")
            self.waypoint_traj.increment_point()

            if self.waypoint_traj.done():
                return self.next_state(context=context)

            segment_point_ij = cartesian_to_ij(context, self.waypoint_traj.get_current_point())
            costmap_length = context.env.cost_map.data.shape[0]
            # If we skipped to a point outside the costmap, begin the spiral again
            # TODO: Behavior should probably to be to go towards the closest point within the costmap
            if not (0 <= int(segment_point_ij[0]) < costmap_length and 0 <= int(segment_point_ij[1]) < costmap_length):
                context.node.get_logger().info("Skipped too far, resetting")
                self.waypoint_traj.reset()
                break

            self.astar_traj.clear()
            return self

        costmap_length = context.env.cost_map.data.shape[0]
        curr_point = cartesian_to_ij(context, self.waypoint_traj.get_current_point())
        if not (0 <= int(curr_point[0]) < costmap_length and 0 <= int(curr_point[1]) < costmap_length):
            context.node.get_logger().warn("Trajectory point out of the map. Clearing trajectory and trying again...")
            self.waypoint_traj.clear()
            return self

        if self.astar_traj.empty():
            self.display_markers(context=context)
            try:
                self.astar_traj = self.astar.generate_trajectory(context, self.waypoint_traj.get_current_point())
            except Exception as e:
                context.node.get_logger().info(str(e))
                return self

            if self.astar_traj.empty():
                context.node.get_logger().info("Skipping unreachable point")
                self.waypoint_traj.increment_point()
                if self.waypoint_traj.done():
                    return self.next_state(context=context)
            # Add extra point for smoother Pure Pursuit
            # Needs to be tested to see if this improves performace
            # elif(self.USE_PURE_PURSUIT):
            #     self.waypoint_traj.increment_point()
            #     if not self.waypoint_traj.done():
            #         np.append(self.astar_traj.coordinates, self.waypoint_traj.get_current_point())
            #     self.waypoint_traj.decerement_point()

            return self

        # Only use pure pursuit if outside of a meter from final target position
        rover_pos = rover_pose.translation()
        target_pos = context.course.current_waypoint_pose_in_map().translation()
        rover_pos[2] = 0
        target_pos[2] = 0
        distance_to_target = np.linalg.norm(target_pos - rover_pos)

        arrived = False
        cmd_vel = Twist()
        if len(self.astar_traj.coordinates) - self.astar_traj.cur_pt != 0:
            waypoint_position_in_map = self.astar_traj.get_current_point()
            cmd_vel, arrived = context.drive.get_drive_command(
                (self.astar_traj if self.USE_PURE_PURSUIT and distance_to_target > 1 else waypoint_position_in_map), # Determine if we are going to use pure pursuit or not
                context.rover.get_pose_in_map(),
                context.node.get_parameter("waypoint.stop_threshold").value,
                context.node.get_parameter("waypoint.drive_forward_threshold").value,
            )

        if arrived:
            self.astar_traj.increment_point()
            if self.astar_traj.done():
                self.astar_traj.clear()
                context.node.get_logger().info(f"Arrived at segment point")
                self.waypoint_traj.increment_point()
                if self.waypoint_traj.done():
                    return self.next_state(context=context)
                self.display_markers(context=context)
        else:
            self.time_no_search_wait = None
            context.rover.send_drive_command(cmd_vel)

        return self

    def on_loop_costmap_disabled(self, context: Context):
        if self.USE_COSTMAP:
            return self
        if context.course is None:
            return state.DoneState()
        if context.course.current_waypoint_pose_in_map() is None:
            return self

        arrived = False
        cmd_vel = Twist()
        cmd_vel, arrived = context.drive.get_drive_command(
            context.course.current_waypoint_pose_in_map().translation(),
            context.rover.get_pose_in_map(),
            context.node.get_parameter("single_tag.stop_threshold").value,
            context.node.get_parameter("waypoint.drive_forward_threshold").value,
        )

        if arrived:
            return self.next_state(context=context)
        else:
            context.rover.send_drive_command(cmd_vel)

        return self

    def on_loop(self, context: Context) -> State:
        """
        Handle driving to a waypoint defined by a linearized cartesian position.
        If the waypoint is associated with a tag id, go into that state early if we see it,
        otherwise wait until we get there to conduct a more thorough search.
        :param context: Context object
        :return:        Next state
        """

        if context.course is None:
            return state.DoneState()

        current_waypoint = context.course.current_waypoint()
        if current_waypoint is None:
            return state.DoneState()

        # If we are at a target currently (from a previous leg), backup to avoid collision
        if context.env.arrived_at_target:
            context.env.arrived_at_target = False
            return backup.BackupState()

        # Returns either ApproachTargetState, LongRangeState, or None
        approach_state = context.course.get_approach_state(use_long_range=False)
        if approach_state is not None:
            return approach_state

        rover_in_map = context.rover.get_pose_in_map()
        if rover_in_map is None:
            return self

        if context.node.get_clock().now() < self.time_begin + Duration(seconds=self.UPDATE_DELAY):
            return self

        if context.rover.stuck:
            context.rover.previous_state = self
            return stuck_recovery.StuckRecoveryState()

        if self.USE_COSTMAP:
            return self.on_loop_costmap_enabled(context)

        else:
            return self.on_loop_costmap_disabled(context)

    def next_state(self, context: Context) -> State:
        if context.course is None:
            return state.DoneState()
        context.env.arrived_at_waypoint = True
        context.node.get_logger().info("Arrived at waypoint")
        context.rover.send_drive_command(Twist())
        current_wp = context.course.current_waypoint()
        if current_wp is None:
            return state.DoneState()
        if current_wp.type.val != WaypointType.NO_SEARCH:
            return costmap_search.CostmapSearchState()
        else:
            if self.time_no_search_wait is None:
                self.time_no_search_wait = context.node.get_clock().now()

            rover_pose = context.rover.get_pose_in_map()
            if rover_pose is None:
                context.node.get_logger().warn("Rover has no pose, waiting...")
                context.rover.send_drive_command(Twist())
                return self

            rover_position = rover_pose.translation()[:2]

            waypoint_position = context.course.current_waypoint_pose_in_map().translation()[:2]
            if (
                context.node.get_clock().now() - self.time_no_search_wait > Duration(seconds=self.NO_SEARCH_WAIT_TIME)
                or d_calc(rover_position, waypoint_position)
                < context.node.get_parameter("waypoint.stop_threshold").value * 2
            ):
                # We finished a regular waypoint, go onto the next one
                if context.course.increment_waypoint():
                    return state.DoneState()
                else:
                    self.on_enter(context=context)
                    return self
            else:
                context.node.get_logger().info("Waiting to get closer to the no search waypoint")
                self.astar_traj.clear()
                self.waypoint_traj.clear()
                return self

    def display_markers(self, context: Context):
        if context.course is None:
            return
        if context.node.get_parameter("display_markers").value:
            start_pt = self.waypoint_traj.cur_pt
            end_pt = min(start_pt + 5, len(self.waypoint_traj.coordinates))
            for i, coord in enumerate(self.waypoint_traj.coordinates[:end_pt]):
                if i >= start_pt:
                    self.marker_pub.publish(
                        gen_marker(
                            time=context.node.get_clock().now(),
                            point=coord,
                            color=[1.0, 0.0, 1.0],
                            id=i,
                            lifetime=context.node.get_parameter("pub_path_rate").value,
                        )
                    )

            if context.course.current_waypoint() is None:
                return

            self.marker_pub.publish(
                gen_marker(
                    time=context.node.get_clock().now(),
                    point=context.course.current_waypoint_pose_in_map().translation()[0:2],
                    color=[0.0, 0.0, 1.0],
                    size=0.5,
                    id=-1,
                    lifetime=10000,
                )
            )
