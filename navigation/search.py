import numpy as np

from mrover.msg import GPSPointList, WaypointType
from state_machine.state import State
from . import recovery, waypoint
from .context import convert_cartesian_to_gps, Context
from .trajectory import SearchTrajectory
from navigation import approach_target


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
