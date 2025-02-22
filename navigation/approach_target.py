import numpy as np

from state_machine.state import State
from . import search, waypoint, state, recovery
from .context import Context


class ApproachTargetState(State):
    def on_enter(self, context: Context) -> None:
        pass

    def on_exit(self, context: Context) -> None:
        pass

    def get_target_position(self, context: Context) -> np.ndarray | None:
        return context.env.current_target_pos()

    def determine_next(self, context: Context, is_finished: bool) -> State:
        if is_finished:
            return state.DoneState()

        if context.rover.stuck:
            context.rover.previous_state = self
            return recovery.RecoveryState()

        return self

    def on_loop(self, context: Context) -> State:
        """
        Drive towards a target based on what gets returned from get_target_position().
        Return to search if there is no target position.
        :return: Next state
        """

        assert context.course is not None

        target_position = self.get_target_position(context)
        if target_position is None:
            from .long_range import LongRangeState

            if isinstance(self, LongRangeState) and not context.env.arrived_at_waypoint:
                return waypoint.WaypointState()

            return search.SearchState()

        rover_in_map = context.rover.get_pose_in_map()
        assert rover_in_map is not None

        cmd_vel, has_arrived = context.drive.get_drive_command(
            target_position,
            rover_in_map,
            context.node.get_parameter("single_tag.stop_threshold").value,
            context.node.get_parameter("waypoint.drive_forward_threshold").value,
        )
        next_state = self.determine_next(context, has_arrived)
        if has_arrived:
            context.env.arrived_at_target = True
            context.env.last_target_location = self.get_target_position(context)
            context.course.increment_waypoint()
        else:
            context.rover.send_drive_command(cmd_vel)

        return next_state
