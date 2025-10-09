from __future__ import annotations

from enum import Enum

import numpy as np

from lie import SE3, normalized, angle_to_rotate_2d
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from trajectory import Trajectory
from typing import overload


class DriveController:
    class DriveMode(Enum):
        TURN_IN_PLACE = 1
        DRIVE_FORWARD = 2
        STOPPED = 3

    _last_angular_error: float | None
    _last_target: np.ndarray | None
    _driver_state: DriveMode

    def __init__(self, node: Node):
        self.node = node
        self._last_angular_error = None
        self._last_target = None
        self._driver_state = self.DriveMode.STOPPED

    def reset(self) -> None:
        self._driver_state = self.DriveMode.STOPPED
        self._last_angular_error = None

    def _get_state_machine_output(
        self,
        angular_error: float,
        linear_error: float,
        completion_thresh: float,
        turn_in_place_thresh: float,
    ) -> tuple[Twist, bool]:
        """
        Gets the state machine output for a given angular and linear error.
        :param angular_error: the angular error to the target
        :param linear_error: the linear error to the target
        :param completion_thresh: the threshold for when we are close enough to the target to consider it reached
        :param turn_in_place_thresh: the threshold for when we should switch from turning in place to driving forward
        :return: a tuple of the command to send to the rover and a boolean indicating whether we are at the target
        :modifies: self._driver_state
        """
        turning_p = self.node.get_parameter("drive.turning_p").value
        driving_p = self.node.get_parameter("drive.driving_p").value
        min_turning_effort = self.node.get_parameter(
            "drive.min_turning_effort").value
        max_turning_effort = self.node.get_parameter(
            "drive.max_turning_effort").value
        min_driving_effort = self.node.get_parameter(
            "drive.min_driving_effort").value
        max_driving_effort = self.node.get_parameter(
            "drive.max_driving_effort").value

        # if we are at the target position, reset the controller and return a zero command
        if abs(linear_error) < completion_thresh:
            self.reset()
            return Twist(), True

        if self._driver_state == self.DriveMode.STOPPED:
            # if the drive mode is STOP (we know we aren't at the target) so we must start moving towards it
            # just switch to the TURN_IN_PLACE state for now under the assumption that we need to turn to face the target
            # return a zero command and False to indicate we aren't at the target (and are also not in the correct state to figure out how to get there)
            self._driver_state = self.DriveMode.TURN_IN_PLACE
            return Twist(), False

        elif self._driver_state == self.DriveMode.TURN_IN_PLACE:
            # if we are in the TURN_IN_PLACE state, we need to turn to face the target
            # if we are within the turn threshold to face the target, we can start driving straight towards it
            if abs(angular_error) < turn_in_place_thresh:
                self._driver_state = self.DriveMode.DRIVE_FORWARD
                return Twist(), False

            # IVT (Intermediate Value Theorem) check. If the sign of the angular error has changed, this means we've crossed through 0 error
            # in order to prevent oscillation, we 'give up' and just switch to the drive forward state
            elif self._last_angular_error is not None and np.sign(self._last_angular_error) != np.sign(angular_error):
                self._driver_state = self.DriveMode.DRIVE_FORWARD
                return Twist(), False

            # if neither of those things are true, we need to turn in place towards our target heading, so set the z component of the output Twist message
            else:
                cmd_vel = Twist(
                    angular=Vector3(
                        z=np.clip(
                            angular_error * turning_p,
                            min_turning_effort,
                            max_turning_effort,
                        )
                    )
                )
                return cmd_vel, False

        elif self._driver_state == self.DriveMode.DRIVE_FORWARD:
            # if we are driving straight towards the target and our last angular error was inside the threshold
            # but our current error was outside the threshold, this means that we have crossed from an acceptable
            # turning error to an unacceptable turning error and we must switch back into the TURN_IN_PLACE state
            # the reason we don't just check if the current error is outside is because it would undermine the IVT
            # check in the TURN_IN_PLACE state and cause oscillation, checking it this way makes it so that we only
            # switch back into the TURN_IN_PLACE state on the "rising edge" of the turn error
            last_angular_was_inside = (
                self._last_angular_error is not None and abs(
                    self._last_angular_error) < turn_in_place_thresh
            )
            cur_angular_is_outside = abs(angular_error) >= turn_in_place_thresh
            if cur_angular_is_outside and last_angular_was_inside:
                self._driver_state = self.DriveMode.TURN_IN_PLACE
                return Twist(), False
            # otherwise we compute a drive command with both a linear and angular component in the Twist message
            else:
                cmd_vel = Twist(
                    linear=Vector3(
                        x=np.clip(
                            linear_error * driving_p,
                            min_driving_effort,
                            max_driving_effort,
                        )
                    ),
                    angular=Vector3(
                        z=np.clip(
                            angular_error * turning_p,
                            min_turning_effort,
                            max_turning_effort,
                        )
                    ),
                )
                return cmd_vel, False
        else:
            raise ValueError(f"Invalid drive state {self._driver_state}")

    @staticmethod
    def compute_lookahead_point(
        path_start: np.ndarray,
        path_end: np.ndarray,
        rover_pos: np.ndarray,
        lookahead_dist: float,
    ) -> np.ndarray:
        """
        Returns a point that the rover should target on the path from the previous target position to the current target position
        The point is computed by first projecting the rovers position onto the path and then propagating ahead by the lookahead distance
        if the target is closer than that then it is returned instead
        :param path_start: the start of the line segment on which to calculate a lookahead point
        :param path_end: the current target position (end of the line segment on which to calculate a lookahead point)
        :param rover_pos: the current rover position
        :param lookahead_dist: the distance to look ahead on the path
        :return: the lookahead point
        """
        # Compute the vector from the previous target position to the current target position
        path_vec = path_end - path_start
        # compute the vector from the previous target position to the rover position
        rover_vec = rover_pos - path_start
        # compute the projection of the rover vector onto the target vector
        proj_vec = np.dot(rover_vec, path_vec) / \
            np.dot(path_vec, path_vec) * path_vec
        lookahead_vec = lookahead_dist * normalized(path_vec)
        lookahead_point = proj_vec + lookahead_vec
        # if the lookahead point is further away than the target, just return the target
        if np.linalg.norm(lookahead_point) > np.linalg.norm(path_vec):
            return path_end
        else:
            return path_start + lookahead_point

    @staticmethod
    def compute_intersection_point(
        waypoints: Trajectory
        rover_pos: np.ndarray
        lookahead_dist: float,
    ) -> np.ndarray:
        #TODO: A helper function for pure pursuit. 
        # Use this to compute the potential point we should be following!

    @overload
    def get_drive_command(
        self: DriveController,
        target_pos: np.ndarray,
        rover_pose: SE3,
        completion_thresh: float,
        turn_in_place_thresh: float,
        drive_back: bool = False,
        path_start: np.ndarray | None = None,
    ) -> tuple[Twist, bool]:
        """
        Returns a drive command to get the rover to the target position, calls the state machine to do so and updates the last angular error in the process
        :param target_pos: The target position to drive to.
        :param rover_pose: The current pose of the rover.
        :param completion_thresh: The distance threshold to consider the rover at the target position.
        :param turn_in_place_thresh: The angle threshold to consider the rover facing the target position and ready to drive forward towards it.
        :param drive_back: True if rover should drive backwards, false otherwise.
        :param path_start: If you want the rover to drive on a line segment (and actively try to stay on the line), pass the start of the line segment as this param, otherwise pass None.
        :return: A tuple of the drive command and a boolean indicating whether the rover is at the target position.
        :modifies: self._last_angular_error
        """

        # Get the direction vector of the rover and the target position,
        # zero the Z components since our controller only assumes motion and control over the rover in the XY plane
        rover_dir = rover_pose.rotation()[:, 0]
        rover_dir[2] = 0

        if drive_back:
            rover_dir *= -1

        rover_pos = rover_pose.translation()
        rover_pos[2] = 0
        target_pos[2] = 0

        if np.linalg.norm(target_pos - rover_pos) < completion_thresh:
            self.reset()
            return Twist(), True

        lookahead_distance = self.node.get_parameter(
            "drive.lookahead_distance").value
        if path_start is not None and np.linalg.norm(path_start - target_pos) > lookahead_distance:
            target_pos = self.compute_lookahead_point(
                path_start, target_pos, rover_pos, lookahead_distance)

        target_dir = target_pos - rover_pos

        # If the target is farther than completion distance away from the last one, reset the controller
        if self._last_target is not None and np.linalg.norm(target_pos - self._last_target) > completion_thresh:
            self.reset()

        # Compute errors
        linear_error = float(np.linalg.norm(target_dir))
        angular_error = angle_to_rotate_2d(rover_dir[:2], target_dir[:2])

        output = self._get_state_machine_output(
            angular_error,
            linear_error,
            completion_thresh,
            turn_in_place_thresh,
        )

        if drive_back:
            output[0].linear.x *= -1

        self._last_angular_error = angular_error
        self._last_target = target_pos
        return output

    @overload
    def get_drive_command(
        self: DriveController,
        waypoints: Trajectory,
        rover_pose: SE3,
        completion_thresh: float,
        turn_in_place_thresh: float,
        drive_back: bool = False,
        path_start: np.ndarray | None = None,
    ) -> tuple[Twist, bool]:
        # TODO: Pure Pursuit Logic: Takes a series of waypoints and the rover's current position
        # and returns a drive command and boolean. The drive command you return should utilize pure
        # pursuit logic: https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit
        # There's a lot of code in the original get_drive_command that could be reused!
        # Maybe put the reusable code in another function?
        return
