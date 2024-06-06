from enum import Enum

import numpy as np

from lie import SO2
from rclpy.duration import Duration
from rclpy.time import Time
from state_machine.state import State
from .context import Context


class JTurnAction(Enum):
    MOVING_BACK = 0
    J_TURNING = 1


class RecoveryState(State):
    waypoint_behind: np.ndarray | None
    current_action: JTurnAction
    start_time: Time | None = None
    waypoint_calculated: bool

    def reset(self, context: Context) -> None:
        self.waypoint_calculated = False
        self.waypoint_behind = None
        self.current_action = JTurnAction.MOVING_BACK
        context.rover.stuck = False
        self.start_time = None

    def on_enter(self, context: Context) -> None:
        self.reset(context)
        self.start_time = context.node.get_clock().now()

    def on_exit(self, context: Context) -> None:
        self.reset(context)

    def on_loop(self, context: Context) -> State:
        give_up_time = context.node.get_parameter("recovery.give_up_time").value
        recovery_distance = context.node.get_parameter("recovery.recovery_distance").value
        stop_thresh = context.node.get_parameter("recovery.stop_threshold").value
        drive_fwd_thresh = (
            context.node.get_parameter("recovery.drive_forward_threshold").value
        )

        if context.node.get_clock().now() - self.start_time > Duration(seconds=give_up_time):
            return context.rover.previous_state

        # Making waypoint behind the rover to go backwards
        rover_in_map = context.rover.get_pose_in_map()
        assert rover_in_map is not None

        # If first round, set a waypoint directly behind the rover and command it to
        # drive backwards toward it until it arrives at that point.
        if self.current_action == JTurnAction.MOVING_BACK:
            # Only set waypoint_behind once so that it doesn't get overwritten and moved
            # further back every iteration
            if self.waypoint_behind is None:
                dir_vector = -1 * recovery_distance * rover_in_map.rotation()[:, 0]
                self.waypoint_behind = rover_in_map.translation() + dir_vector

            cmd_vel, arrived_back = context.drive.get_drive_command(
                self.waypoint_behind, rover_in_map, stop_thresh, drive_fwd_thresh, drive_back=True
            )
            context.rover.send_drive_command(cmd_vel)

            if arrived_back:
                self.current_action = JTurnAction.J_TURNING  # move to second part of turn
                self.waypoint_behind = None
                context.drive.reset()

        # If second round, set a waypoint off to the side of the rover and command it to
        # turn and drive backwards towards it until it arrives at that point. So it will begin
        # by turning then it will drive backwards.
        if self.current_action == JTurnAction.J_TURNING:
            if self.waypoint_behind is None:
                dir_vector = rover_in_map.rotation()[:, 0]
                # The waypoint will be 45 degrees to the left of the rover behind it.
                dir_vector[:2] = recovery_distance * SO2(3 * np.pi / 4).act(dir_vector[:2])
                self.waypoint_behind = rover_in_map.translation() + dir_vector

            cmd_vel, arrived_turn = context.drive.get_drive_command(
                self.waypoint_behind, rover_in_map, stop_thresh, drive_fwd_thresh, drive_back=True
            )
            context.rover.send_drive_command(cmd_vel)

            # set stuck to False
            if arrived_turn:
                return context.rover.previous_state

        return self
