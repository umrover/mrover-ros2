import numpy as np
from typing import Optional
from navigation.coordinate_utils import d_calc
from mrover.msg import GPSPointList, WaypointType
from state_machine.state import State
from . import recovery, waypoint
from navigation.coordinate_utils import gen_marker
from .context import convert_cartesian_to_gps, Context
from .trajectory import SearchTrajectory
from navigation import approach_target
from rclpy.time import Time
from rclpy.duration import Duration
from visualization_msgs.msg import Marker
from rclpy.publisher import Publisher


class SearchState(State):
    trajectory: SearchTrajectory | None = None
    prev_target_pos_in_map: np.ndarray | None = None
    is_recovering: bool = False
    prev_pos: Optional[np.ndarray]
    time_begin: Optional[Time]
    time_last_updated: Time
    total_distance: float
    marker_pub: Publisher

    def on_enter(self, context: Context) -> None:
        if SearchState.trajectory is None:
            self.new_trajectory(context)
        self.marker_pub = context.node.create_publisher(Marker, "spiral_points", 10)
        self.prev_pos = None
        self.time_begin = None
        self.time_last_updated = context.node.get_clock().now()
        self.total_distance = 0.0

    def on_exit(self, context: Context) -> None:
        pass

    def on_loop(self, context: Context) -> State:
        rover_in_map = context.rover.get_pose_in_map()

        assert rover_in_map is not None
        assert SearchState.trajectory is not None


        if not self.time_begin:
            self.time_begin = context.node.get_clock().now()
            self.prev_pos = rover_in_map.translation()[0:2]
        else:
            self.total_distance += d_calc(rover_in_map.translation()[0:2], self.prev_pos)
            self.prev_pos = rover_in_map.translation()[0:2]
        
        if context.node.get_clock().now() - self.time_last_updated > Duration(seconds=3.0):
            total_time = (context.node.get_clock().now() - self.time_begin).nanoseconds / 1e9
            context.node.get_logger().info(f"Total Distance Traveled: {self.total_distance}m\nTotal Time: {total_time}s\nAverage Speed: {self.total_distance/total_time}m/s")
            self.time_last_updated = context.node.get_clock().now()
            start_pt = self.trajectory.cur_pt - 6 if self.trajectory.cur_pt - 3 >= 0 else self.trajectory.cur_pt
            end_pt = self.trajectory.cur_pt + 6 if self.trajectory.cur_pt + 3 < len(self.trajectory.coordinates) else len(self.trajectory.coordinates)
            for i, coord in enumerate(self.trajectory.coordinates[start_pt:end_pt]):
                self.marker_pub.publish(gen_marker(context=context, point=coord, color=[1.0,0.0,0.0], id=i))
        
        # Continue executing the path from wherever it left off
        target_position_in_map = SearchState.trajectory.get_current_point()
        cmd_vel, arrived = context.drive.get_drive_command(
            target_position_in_map,
            rover_in_map,
            context.node.get_parameter("search.stop_threshold").value,
            context.node.get_parameter("search.drive_forward_threshold").value,
            path_start=self.prev_target_pos_in_map,
        )
        if arrived:
            if target_position_in_map is not None and self.prev_target_pos_in_map is not None:
                context.node.get_logger().info(str(np.sqrt((target_position_in_map[0] - self.prev_target_pos_in_map[0]) ** 2 + (target_position_in_map[1] - self.prev_target_pos_in_map[1]) ** 2)))

            self.prev_target_pos_in_map = target_position_in_map
            # If we finish the spiral without seeing the tag, move on with course
            if SearchState.trajectory.increment_point():
                return waypoint.WaypointState()

        if context.rover.stuck:
            context.rover.previous_state = self
            self.is_recovering = True
            return recovery.RecoveryState()
        else:
            self.is_recovering = False

        # ref = np.array(
        #     [
        #         context.node.get_parameter("ref_lat").value,
        #         context.node.get_parameter("ref_lon").value,
        #         context.node.get_parameter("ref_alt").value,
        #     ]
        # )
        # context.search_point_publisher.publish(
        #     GPSPointList(points=[convert_cartesian_to_gps(ref, p) for p in SearchState.trajectory.coordinates])
        # )
        context.rover.send_drive_command(cmd_vel)

        # Returns either ApproachTargetState, LongRangeState, or None
        assert context.course is not None
        #approach_state = context.course.get_approach_state()
        
        if context.env.current_target_pos() is not None:
            return approach_target.ApproachTargetState()

        return self

    def new_trajectory(self, context) -> None:
        if self.is_recovering:
            return

        assert context.course is not None
        search_center = context.course.current_waypoint()

        if search_center.type.val == WaypointType.POST:
            SearchState.trajectory = SearchTrajectory.spiral_traj(
                context.course.current_waypoint_pose_in_map().translation()[0:2],
                context.node.get_parameter("search.coverage_radius").value,
                context.node.get_parameter("search.distance_between_spirals").value,
                context.node.get_parameter("search.segments_per_rotation").value,
                context.node.get_parameter("search.max_segment_length").value,
                search_center.tag_id,
                False,
            )
        else:  # water bottle or mallet
            SearchState.trajectory = SearchTrajectory.spiral_traj(
                context.course.current_waypoint_pose_in_map().translation()[0:2],
                context.node.get_parameter("search.coverage_radius").value,
                context.node.get_parameter("search.distance_between_spirals").value,
                context.node.get_parameter("search.segments_per_rotation").value,
                context.node.get_parameter("search.max_segment_length").value,
                search_center.tag_id,
                False,
            )
        self.prev_target_pos_in_map = None
