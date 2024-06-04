from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from shapely.geometry import Point, LineString

from geometry import SE3, perpendicular_2d
from state_machine.state import State
from . import waypoint, recovery
from .context import Context
from .trajectory import Trajectory


@dataclass
class AvoidPostTrajectory(Trajectory):
    @staticmethod
    def avoid_post_trajectory(
        context: Context, rover_pose: SE3, post_pos: np.ndarray, waypoint_pos: np.ndarray
    ) -> AvoidPostTrajectory:
        """
        Generates a trajectory that avoids a post until the rover has a clear path to the waypoint
        :param context:         State machine context
        :param rover_pose:      The current pose of the rover
        :param post_pos:        The position of the post
        :param waypoint_pos:    The position of the waypoint
        :return:                A trajectory that avoids the post, including the first point as a backup point that the rover MUST drive backwards towards

        Summary on how trajectory works:
        we first generate a backup point that is directly behind
        the rover some BACKUP_DISTANCE behind it. Then we see if
        we can drive directly to the waypoint without intersecting
        a circle of size POST_RADIUS ( < BACKUP_DISTANCE)
        if we can then the trajectory is just the backup point.
        If we cannot then we generate an avoidance point BACKUP_DISTANCE away
        from the post at a 90 degree angle from the rover relative to the post
        either to the left or the right depending on which is closer to the waypoint.
        Then the trajectory is backup_point, avoidance_point where we drive backwards to
        the backup_point and forwards to the avoidance_point.
        """
        rover_pos = rover_pose.position
        rover_direction = rover_pose.rotation()[:, 0]

        # Converting to 2D arrays
        post_pos = post_pos[:2]
        rover_pos = rover_pos[:2]
        rover_direction = rover_direction[:2]
        waypoint_pos = waypoint_pos[:2]

        rover_direction = rover_direction / np.linalg.norm(rover_direction)

        post_radius = context.node.get_parameter("single_tag/post_radius").get_parameter_value().double_value
        post_circle = Point(post_pos[0], post_pos[1]).buffer(post_radius)

        backup_distance = context.node.get_parameter("recovery/recovery_distance").get_parameter_value().double_value
        backup_point = rover_pos - backup_distance * rover_direction

        path = LineString([backup_point, waypoint_pos])

        # Check if the path intersects the post circle
        if path.intersects(post_circle):
            # Get a vector perpendicular to vector from rover to post
            rover_to_post = post_pos - backup_point
            rover_to_post = rover_to_post / np.linalg.norm(rover_to_post)
            left_perp = perpendicular_2d(rover_to_post)  # (-y,x)
            right_perp = -left_perp
            avoidance_point_left = post_pos + backup_distance * left_perp
            avoidance_point_right = post_pos + backup_distance * right_perp
            left_dist = np.linalg.norm(avoidance_point_left - waypoint_pos)
            right_dist = np.linalg.norm(avoidance_point_right - waypoint_pos)
            if left_dist < right_dist:
                coords = np.array([backup_point, avoidance_point_left])
            else:
                coords = np.array([backup_point, avoidance_point_right])
        else:
            coords = np.array([backup_point])

        # add a z coordinate of 0 to all the coords
        coords = np.hstack((coords, np.zeros((coords.shape[0], 1))))
        return AvoidPostTrajectory(coords)


class PostBackupState(State):
    trajectory: AvoidPostTrajectory | None

    def on_exit(self, context: Context) -> None:
        self.trajectory = None

    def on_enter(self, context: Context) -> None:
        assert context.course is not None

        if context.env.last_target_location is None:
            self.trajectory = None
            return

        rover_in_map = context.rover.get_pose_in_map()
        assert rover_in_map is not None

        self.trajectory = AvoidPostTrajectory.avoid_post_trajectory(
            rover_in_map,
            context.env.last_target_location,
            context.course.current_waypoint_pose_in_map().position,
        )
        self.trajectory.cur_pt = 0

    def on_loop(self, context: Context) -> State:
        if self.trajectory is None:
            return waypoint.WaypointState()

        target_pos = self.trajectory.get_current_point()

        # we drive backwards to the first point in this trajectory
        point_index = self.trajectory.cur_pt
        drive_backwards = point_index == 0

        rover_in_map = context.rover.get_pose_in_map()
        assert rover_in_map is not None

        stop_thresh = context.node.get_parameter("search/stop_threshold").get_parameter_value().double_value
        drive_fwd_thresh = (
            context.node.get_parameter("search/drive_forward_threshold").get_parameter_value().double_value
        )
        cmd_vel, arrived = context.drive.get_drive_command(
            target_pos,
            rover_in_map,
            stop_thresh,
            drive_fwd_thresh,
            drive_back=drive_backwards,
        )
        if arrived:
            context.node.get_logger().info(f"Arrived at point indexed: {point_index}")
            if self.trajectory.increment_point():
                self.trajectory = None
                return waypoint.WaypointState()

        if context.rover.stuck:
            context.rover.previous_state = self
            self.trajectory = None
            return recovery.RecoveryState()

        context.rover.send_drive_command(cmd_vel)
        return self
