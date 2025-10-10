import numpy as np

from lie import SO2
from rclpy.duration import Duration
from rclpy.time import Time
from state_machine.state import State
from .context import Context
from .trajectory import Trajectory
from .coordinate_utils import is_high_cost_point


class PathHistoryExhausted(Exception):
    """
    Raised when the rover has run out of points in its path history to backtrack on
    """

    pass


class HighCostRecoveryState(State):
    backtrack_traj: Trajectory
    STOP_THRESH: float
    DRIVE_FWD_THRESH: float

    def on_enter(self, context: Context) -> None:
        assert context.rover.path_history is not None
        context.node.get_logger().info("Entered HighCostRecoveryState")
        reverse_path = np.array(
            [
                [pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z]
                for pose_stamped in context.rover.path_history
            ]
        )[::-1]
        self.backtrack_traj = Trajectory(reverse_path)

        self.STOP_THRESH = context.node.get_parameter("recovery.stop_threshold").value
        self.DRIVE_FWD_THRESH = context.node.get_parameter("recovery.drive_forward_threshold").value

    def on_exit(self, context: Context) -> None:
        pass

    def on_loop(self, context: Context) -> State:
        rover_pose = context.rover.get_pose_in_map()
        assert rover_pose is not None

        rover_position = rover_pose.translation()

        if not is_high_cost_point(point=rover_position, context=context):
            return context.rover.previous_state

        cmd_vel, arrived = context.drive.get_drive_command(
            self.backtrack_traj.get_current_point(),
            rover_pose,
            self.STOP_THRESH,
            self.DRIVE_FWD_THRESH,
            drive_back=True,
        )

        if arrived:
            self.backtrack_traj.increment_point()
            if self.backtrack_traj.done():
                raise PathHistoryExhausted
        else:
            context.rover.send_drive_command(cmd_vel)

        return self
