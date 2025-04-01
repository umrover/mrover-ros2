from typing import Optional
import numpy as np

import rclpy
from rclpy.publisher import Publisher
from rclpy.time import Time
from rclpy.duration import Duration
import time
from navigation import approach_target, recovery, waypoint
from navigation.astar import AStar, SpiralEnd, NoPath
from navigation.coordinate_utils import d_calc, gen_marker, is_high_cost_point
from navigation.context import Context
from navigation.trajectory import Trajectory, SearchTrajectory
from visualization_msgs.msg import Marker
from state_machine.state import State
from rclpy.publisher import Publisher


# REFERENCE: https://docs.google.com/document/d/18GjDWxIu5f5-N5t5UgbrZGdEyaDj9ZMEUuXex8-NKrA/edit
class CostmapSearchState(State):
    """
    General search state that utilizes the costmap for searching for any obstacles
    Follows a search spiral but uses A* to avoid obstacles
    Intended to be implemented over the current search state
    """

    prev_pos: np.ndarray
    time_begin: Optional[Time]
    total_distance: float

    traj: SearchTrajectory
    star_traj: Trajectory  # returned by astar
    prev_target_pos_in_map: Optional[np.ndarray] = None
    is_recovering: bool = False
    time_last_updated: Time
    path_pub: Publisher
    astar: AStar
    marker_pub: Publisher

    STOP_THRESH: float
    DRIVE_FWD_THRESH: float
    UPDATE_DELAY: float

    def on_enter(self, context: Context) -> None:
        context.node.get_logger().info("In costmap search spiral state")

        # Parameter initialization
        self.prev_pos = np.array([0, 0])
        self.time_begin = None
        self.total_distance = 0.0
        self.marker_pub = context.node.create_publisher(Marker, "spiral_points", 10)
        self.STOP_THRESH = context.node.get_parameter("search.stop_threshold").value
        self.DRIVE_FWD_THRESH = context.node.get_parameter("search.drive_forward_threshold").value
        self.UPDATE_DELAY = context.node.get_parameter("search.update_delay").value
        self.A_STAR_THRESH = context.node.get_parameter("search.a_star_thresh").value

        # Creates spiral traj to follow
        self.new_traj(context)

        if not self.is_recovering:
            self.prev_target_pos_in_map = None

        # Gets a-star ready
        self.astar = AStar(context)
        self.star_traj = Trajectory(np.array([]))
        self.time_last_updated = context.node.get_clock().now()
        self.follow_astar = False

        # Initialize stopwatch
        self.time_begin = None

    def on_exit(self, context: Context) -> None:
        pass

    def on_loop(self, context: Context) -> State:
        # Wait until the costmap is ready
        if not hasattr(context.env.cost_map, "data"):
            context.node.get_logger().warn(f"No costmap found, waiting...")
            time.sleep(1.0)
            return self

        assert context.course is not None
        assert self.prev_pos is not None

        rover_in_map = context.rover.get_pose_in_map()
        assert rover_in_map is not None

        if context.rover.stuck:
            context.rover.previous_state = self
            self.is_recovering = True
            return recovery.RecoveryState()
        else:
            self.is_recovering = False

        if not self.time_begin:
            self.time_begin = context.node.get_clock().now()
            self.prev_pos = rover_in_map.translation()[0:2]

        else:
            self.total_distance += d_calc(rover_in_map.translation()[0:2], tuple(self.prev_pos))
            self.prev_pos = rover_in_map.translation()[0:2]

        while is_high_cost_point(context=context, point=self.traj.get_current_point()):
            context.node.get_logger().info(f"Skipping high cost spiral point")
            if self.traj.increment_point():
                context.node.get_logger().info(f"Reached end of search spiral")
                return waypoint.WaypointState()
            start_pt = self.traj.cur_pt
            end_pt = (
                self.traj.cur_pt + 6
                if self.traj.cur_pt + 3 < len(self.traj.coordinates)
                else len(self.traj.coordinates)
            )
            if context.node.get_parameter("search.display_markers").value:
                for i, coord in enumerate(self.traj.coordinates[start_pt:end_pt]):
                    self.marker_pub.publish(
                        gen_marker(context=context, point=coord, color=[1.0, 0.0, 0.0], id=i, lifetime=100)
                    )
            self.star_traj = Trajectory(np.array([]))

        # If there are no more points in the current a_star path or we are past the update delay, then create a new one
        if len(
            self.star_traj.coordinates
        ) - self.star_traj.cur_pt == 0 or context.node.get_clock().now() - self.time_last_updated > Duration(
            seconds=self.UPDATE_DELAY
        ):

            start_pt = self.traj.cur_pt
            end_pt = (
                self.traj.cur_pt + 6
                if self.traj.cur_pt + 3 < len(self.traj.coordinates)
                else len(self.traj.coordinates)
            )
            if context.node.get_parameter("search.display_markers").value:
                for i, coord in enumerate(self.traj.coordinates[start_pt:end_pt]):
                    self.marker_pub.publish(
                        gen_marker(context=context, point=coord, color=[1.0, 0.0, 0.0], id=i, lifetime=100)
                    )

            total_time = (context.node.get_clock().now() - self.time_begin).nanoseconds / 1e9
            # context.node.get_logger().info(f"Total Distance Traveled: {self.total_distance}m\nTotal Time: {total_time}s\nAverage Speed: {self.total_distance/total_time}m/s")

            # Generate a path
            self.star_traj = self.astar.generate_trajectory(context, self.traj.get_current_point())

            # edge case, if for some reason we cannot astar, skip the point and attempt to astar to the next
            while len(self.star_traj.coordinates) == 0:
                context.node.get_logger().info(f"Skipping spiral point")
                if self.traj.increment_point():
                    context.node.get_logger().info(f"Reached end of search spiral")
                    return waypoint.WaypointState()
                self.star_traj = self.astar.generate_trajectory(context, self.traj.get_current_point())

            self.time_last_updated = context.node.get_clock().now()

        target_position_in_map = self.star_traj.get_current_point()

        cmd_vel, arrived = context.drive.get_drive_command(
            target_position_in_map,
            rover_in_map,
            self.STOP_THRESH,
            self.DRIVE_FWD_THRESH,
            path_start=self.prev_target_pos_in_map,
        )

        if not arrived:
            context.rover.send_drive_command(cmd_vel)
        else:
            self.prev_target_pos_in_map = target_position_in_map
            if self.star_traj.increment_point():
                if self.traj.increment_point():
                    context.node.get_logger().info(f"Reached end of search spiral")
                    return waypoint.WaypointState()

        # If our target object has been detected, approach it
        if (context.env.current_target_pos()) is not None:
            total_time = context.node.get_clock().now() - self.time_begin
            context.node.get_logger().info(f"Total search time: {total_time.nanoseconds // 1000000000}")
            return approach_target.ApproachTargetState()

        return self

    def new_traj(self, context) -> None:
        assert context.course is not None
        search_center = context.course.current_waypoint()
        assert search_center is not None

        if not self.is_recovering:
            self.traj = SearchTrajectory.spiral_traj(
                center=context.course.current_waypoint_pose_in_map().translation()[0:2],
                coverage_radius=context.node.get_parameter("search.coverage_radius").value,
                distance_between_spirals=context.node.get_parameter("search.distance_between_spirals").value,
                segments_per_rotation=context.node.get_parameter("search.segments_per_rotation").value,
                max_segment_length=context.node.get_parameter("search.max_segment_length").value,
                tag_id=search_center.tag_id,
                insert_extra=True,
            )
