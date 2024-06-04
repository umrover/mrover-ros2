import numpy as np

from mrover.msg import GPSPointList, WaypointType
from state_machine.state import State
from . import recovery, waypoint
from .context import convert_cartesian_to_gps, Context
from .trajectory import SearchTrajectory


class SearchState(State):
    trajectory: SearchTrajectory | None = None
    prev_target_pos_in_map: np.ndarray | None = None
    is_recovering: bool = False

    def on_enter(self, context: Context) -> None:
        if SearchState.trajectory is None:
            self.new_trajectory(context)

    def on_exit(self, context: Context) -> None:
        pass

    def on_loop(self, context: Context) -> State:
        rover_in_map = context.rover.get_pose_in_map()

        assert rover_in_map is not None
        assert SearchState.trajectory is not None

        # Continue executing the path from wherever it left off
        target_position_in_map = SearchState.trajectory.get_current_point()
        cmd_vel, arrived = context.drive.get_drive_command(
            target_position_in_map,
            rover_in_map,
            context.node.get_parameter("search/stop_threshold").get_parameter_value().double_value,
            context.node.get_parameter("search/drive_forward_threshold").get_parameter_value().double_value,
            path_start=self.prev_target_pos_in_map,
        )
        if arrived:
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

        ref = np.array(
            [
                context.node.get_parameter("gps_linearization/ref_lat").get_parameter_value().double_value,
                context.node.get_parameter("gps_linearization/ref_long").get_parameter_value().double_value,
                context.node.get_parameter("gps_linearization/ref_alt").get_parameter_value().double_value,
            ]
        )
        context.search_point_publisher.publish(
            GPSPointList(points=[convert_cartesian_to_gps(ref, p) for p in SearchState.trajectory.coordinates])
        )
        context.rover.send_drive_command(cmd_vel)

        # Returns either ApproachTargetState, LongRangeState, or None
        assert context.course is not None
        approach_state = context.course.get_approach_state()
        if approach_state is not None:
            return approach_state

        return self

    def new_trajectory(self, context) -> None:
        if self.is_recovering:
            return

        assert context.course is not None
        search_center = context.course.current_waypoint()

        if search_center.type.val == WaypointType.POST:
            SearchState.trajectory = SearchTrajectory.spiral_traj(
                context.course.current_waypoint_pose_in_map().position[0:2],
                context.node.get_parameter("search/coverage_radius").get_parameter_value().double_value,
                context.node.get_parameter("search/distance_between_spirals").get_parameter_value().double_value,
                context.node.get_parameter("search/segments_per_rotation").get_parameter_value().integer_value,
                search_center.tag_id,
                False,
            )
        else:  # water bottle or mallet
            SearchState.trajectory = SearchTrajectory.spiral_traj(
                context.course.current_waypoint_pose_in_map().position[0:2],
                context.node.get_parameter("object_search/coverage_radius").get_parameter_value().double_value,
                context.node.get_parameter("object_search/distance_between_spirals").get_parameter_value().double_value,
                context.node.get_parameter("search/segments_per_rotation").get_parameter_value().integer_value,
                search_center.tag_id,
                False,
            )
        self.prev_target_pos_in_map = None
