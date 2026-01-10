import numpy as np

from lie import SO2
from rclpy.duration import Duration
from rclpy.time import Time
from state_machine.state import State
from . import state, waypoint
from .context import Context
from .trajectory import Trajectory
from .coordinate_utils import is_high_cost_point
from geometry_msgs.msg import Twist, PoseStamped, Point


class BackupState(State):
    backtrack_traj: Trajectory
    STOP_THRESH: float
    DRIVE_FWD_THRESH: float
    BACKUP_DIST: float
    WAIT_TIME: float
    dist_traveled: float
    prev_pos: np.ndarray
    start_time: Time

    def on_enter(self, context: Context) -> None:
        context.node.get_logger().info("Entered Backup State")

        if context.rover.path_history is None or len(context.rover.path_history.poses) < 1:
            context.node.get_logger().warn("Cannot backup: no path history")
            return

        path_history_poses: list[PoseStamped] = context.rover.path_history.poses
        path_history_poses.reverse()
        path_history_coordinates = np.array(
            [
                [pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z]
                for pose_stamped in path_history_poses
            ]
        )

        self.backtrack_traj = Trajectory(path_history_coordinates)
        self.dist_traveled = 0.0
        self.start_time = context.node.get_clock().now()

        rover_pose = context.rover.get_pose_in_map()
        if rover_pose is None:
            return
        self.prev_pos = rover_pose.translation()[0:2]

        self.STOP_THRESH = context.node.get_parameter("backup.stop_threshold").value
        self.DRIVE_FWD_THRESH = context.node.get_parameter("backup.drive_forward_threshold").value
        self.BACKUP_DIST = context.node.get_parameter("backup.backup_distance").value
        self.WAIT_TIME = context.node.get_parameter("backup.wait_time").value

    def on_exit(self, context: Context) -> None:
        pass

    def next_state(self, context: Context) -> State:
        if context.course is None or context.course.done():
            return state.DoneState()

        return waypoint.WaypointState()

    def on_loop(self, context: Context) -> State:
        if len(context.rover.path_history.poses) == 0 or self.backtrack_traj.done():
            return self.next_state(context)

        if context.node.get_clock().now() - self.start_time < Duration(seconds=self.WAIT_TIME):
            return self

        rover_pose = context.rover.get_pose_in_map()

        if rover_pose is None:
            context.node.get_logger().warn("Rover has no pose, waiting...")
            context.rover.send_drive_command(Twist())
            return self

        rover_position = rover_pose.translation()[0:2]

        self.dist_traveled += float(np.linalg.norm(rover_position - self.prev_pos))
        self.prev_pos = rover_position
        if self.dist_traveled > self.BACKUP_DIST:
            return self.next_state(context)

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
                return self.next_state(context)
        else:
            context.rover.send_drive_command(cmd_vel)

        return self
