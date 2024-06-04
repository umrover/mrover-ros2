from state_machine.state import State
from . import (
    search,
    recovery,
    post_backup,
    state,
)
from .context import Context


class WaypointState(State):
    # STOP_THRESHOLD: float = rospy.get_param("waypoint/stop_threshold")
    # DRIVE_FORWARD_THRESHOLD: float = rospy.get_param("waypoint/drive_forward_threshold")
    # USE_COSTMAP: bool = rospy.get_param("water_bottle_search/use_costmap")
    # NO_TAG: int = -1

    def on_enter(self, context: Context) -> None:
        assert context.course is not None

        current_waypoint = context.course.current_waypoint()
        assert current_waypoint is not None

        context.env.arrived_at_waypoint = False

    def on_exit(self, context: Context) -> None:
        pass

    def on_loop(self, context: Context) -> State:
        """
        Handle driving to a waypoint defined by a linearized cartesian position.
        If the waypoint is associated with a tag id, go into that state early if we see it,
        otherwise wait until we get there to conduct a more thorough search.
        :param context: Context object
        :return:        Next state
        """
        assert context.course is not None

        current_waypoint = context.course.current_waypoint()
        if current_waypoint is None:
            return state.DoneState()

        # If we are at a post currently (from a previous leg), backup to avoid collision
        if context.env.arrived_at_target:
            context.env.arrived_at_target = False
            return post_backup.PostBackupState()

        # Returns either ApproachTargetState, LongRangeState, or None
        approach_state = context.course.get_approach_state()
        if approach_state is not None:
            return approach_state

        rover_in_map = context.rover.get_pose_in_map()
        if rover_in_map is None:
            return self

        # Attempt to find the waypoint in the TF tree and drive to it
        waypoint_position_in_map = context.course.current_waypoint_pose_in_map().translation()
        cmd_vel, arrived = context.drive.get_drive_command(
            waypoint_position_in_map,
            rover_in_map,
            context.node.get_parameter("waypoint/stop_threshold").value,
            context.node.get_parameter("waypoint/drive_forward_threshold").value,
        )
        if arrived:
            context.env.arrived_at_waypoint = True
            if context.course.look_for_post() or context.course.look_for_object():
                # We finished a waypoint associated with a post, mallet, or water bottle, but we have not seen it yet
                search_state = search.SearchState()
                search_state.new_trajectory(context)  # reset trajectory
                return search_state
            else:
                # We finished a regular waypoint, go onto the next one
                context.course.increment_waypoint()

        if context.rover.stuck:
            context.rover.previous_state = self
            return recovery.RecoveryState()

        context.rover.send_drive_command(cmd_vel)

        return self
