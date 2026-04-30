from __future__ import annotations

from enum import Enum

import numpy as np

from lie import SE3, normalized, angle_to_rotate_2d
from geometry_msgs.msg import Twist, Vector3
from navigation.marker_utils import gen_marker, ring_marker
from rclpy.node import Node
from rclpy.publisher import Publisher
from trajectory import Trajectory
from visualization_msgs.msg import Marker


class DriveController:
    class DriveMode(Enum):
        TURN_IN_PLACE = 1
        DRIVE_FORWARD = 2
        STOPPED = 3

    _last_angular_error: float | None
    _last_target: np.ndarray | None
    _last_lookahead_dist: float
    _driver_state: DriveMode
    lookahead_pub: Publisher
    intersection_pub: Publisher
    USE_PURE_PURSUIT: bool

    def __init__(self, node: Node, lookahead_pub: Publisher, intersect_pub: Publisher):
        self.node = node
        self._last_angular_error = None
        self._last_target = None
        self._last_lookahead_dist = self.node.get_parameter("pure_pursuit.min_lookahead_distance").value
        self._last_point = None
        self.USE_PURE_PURSUIT = self.node.get_parameter_or("pure_pursuit.use_pure_pursuit", True).value
        self.lookahead_pub = lookahead_pub
        self.intersection_pub = intersect_pub
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
        min_turning_effort = self.node.get_parameter("drive.min_turning_effort").value
        max_turning_effort = self.node.get_parameter("drive.max_turning_effort").value
        min_driving_effort = self.node.get_parameter("drive.min_driving_effort").value
        max_driving_effort = self.node.get_parameter("drive.max_driving_effort").value

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
                self._last_angular_error is not None and abs(self._last_angular_error) < turn_in_place_thresh
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
        proj_vec = np.dot(rover_vec, path_vec) / np.dot(path_vec, path_vec) * path_vec
        lookahead_vec = lookahead_dist * normalized(path_vec)
        lookahead_point = proj_vec + lookahead_vec
        # if the lookahead point is further away than the target, just return the target
        if np.linalg.norm(lookahead_point) > np.linalg.norm(path_vec):
            return path_end
        else:
            return path_start + lookahead_point

    def get_turn_command(
        self: DriveController,
        target_dir: np.ndarray,
        rover_pose: SE3,
        completion_thresh: float,
    ) -> tuple[Twist, bool]:
        turning_p = self.node.get_parameter("drive.turning_p").value
        min_turning_effort = self.node.get_parameter("drive.min_turning_effort").value
        max_turning_effort = self.node.get_parameter("drive.max_turning_effort").value

        rover_dir = rover_pose.rotation()[:, 0]
        rover_dir[2] = 0

        angular_error: float = angle_to_rotate_2d(target_dir[:2], rover_dir[:2])
        if abs(angular_error) < completion_thresh:
            return Twist(), True

        return (
            Twist(
                angular=Vector3(
                    z=np.clip(
                        -angular_error * turning_p,
                        min_turning_effort,
                        max_turning_effort,
                    )
                )
            ),
            False,
        )

    def get_drive_command(
        self: DriveController,
        target_pos: np.ndarray | Trajectory,
        rover_pose: SE3,
        completion_thresh: float,
        turn_in_place_thresh: float,
        drive_back: bool = False,
        path_start: np.ndarray | None = None,
        last_point: bool = True,
    ) -> tuple[Twist, bool]:
        """
        :param target_pos: The target point of the rover. If using pure pursuit, use target_pos for the astar_traj and use full_traj for the full trajectory.
        :param rover_pose: The current pose of the rover.
        :param completion_thresh: The distance threshold to consider the rover at the target position.
        :param turn_in_place_thresh: The angle threshold to consider the rover facing the target position and ready to drive forward towards it.
        :param drive_back: True if rover should drive backwards, false otherwise.
        :param path_start: If you want the rover to drive on a line segment (and actively try to stay on the line), pass the start of the line segment as this param, otherwise pass None.
        :param last_point: Used for pure pursuit to determine if the rover is at the end of its path.
        :return: A tuple of the drive command and a boolean indicating whether the rover is at the target position.
        """
        # If target_pos is a Trajectory we use pure pursuit instead of default_drive_command
        if isinstance(target_pos, np.ndarray):
            return self.get_default_drive_command(
                target_pos,
                rover_pose,
                completion_thresh,
                turn_in_place_thresh,
                drive_back,
                path_start,
            )
        # The Pure Pursuit is known for its trouble to arrive at a given point, it is mainly used for paths but not arriving at a point.
        # Adding this condition makes the controller use the default drive command, fixing this issue, and is the reason why the whole trajectory (full_traj) is needed (so we know the end)
        elif not self.USE_PURE_PURSUIT or drive_back or (target_pos.is_last() and last_point):
            return self.get_default_drive_command(
                target_pos.get_current_point(),
                rover_pose,
                completion_thresh,
                turn_in_place_thresh,
                drive_back,
                path_start,
            )
        return self.get_pure_pursuit_drive_command(
            target_pos,
            rover_pose,
        )

    def get_default_drive_command(
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

        # Set the pure pursuit lookahead distance to the minimum incase it will be used again
        self._last_lookahead_dist = self.node.get_parameter("pure_pursuit.min_lookahead_distance").value

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

        lookahead_distance = self.node.get_parameter("drive.lookahead_distance").value
        if path_start is not None and np.linalg.norm(path_start - target_pos) > lookahead_distance:
            target_pos = self.compute_lookahead_point(path_start, target_pos, rover_pos, lookahead_distance)

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

    def get_pure_pursuit_drive_command(
        self: DriveController,
        waypoints: Trajectory,
        rover_pose: SE3,
    ) -> tuple[Twist, bool]:
        """
        Returns a drive command to get the rover to the target position, using pure pursuit logic
        :param waypoints: The current trajectory of the rover.
        :param rover_pose: The current pose of the rover.
        :param completion_thresh: The distance threshold to consider the rover at the target position.
        :param drive_back: True if rover should drive backwards, false otherwise.
        :return: A tuple of the drive command and a boolean indicating whether the rover is at the target position.
        :modifies: self._last_lookahead_dist
        """
        # Pure Pursuit Logic: Takes a series of waypoints and the rover's current position
        # and returns a drive command and boolean. The drive command you return should utilize pure
        # pursuit logic: https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit

        # Trajectory is None -> There are no points to go to (this should never occur if this function is used correctly)
        assert waypoints is not None

        # Get the direction vector of the rover and position,
        # zero the Z components since our controller only assumes motion and control over the rover in the XY plane
        rover_dir = rover_pose.rotation()[:, 0]
        rover_dir[2] = 0
        rover_pos = rover_pose.translation()
        rover_pos[2] = 0

        # Check and set if there is a new farther found point in the path
        self.set_farthest_path_point(self, waypoints, rover_pos)

        # If we are at the end of the traj, return a zero command
        if waypoints.done():
            return Twist(), True

        # Compute intersection points with the path
        intersection_points = self.compute_intersection_point(waypoints, rover_pos)

        # Determine the target_pos given the intersection points
        target_pos = self.determine_next_point(waypoints, intersection_points)

        self.display_markers(rover_pos, target_pos)

        # Get target direction vector
        target_dir = target_pos - rover_pos

        # Compute angular error to target_dir
        angular_error = angle_to_rotate_2d(rover_dir[:2], target_dir[:2])

        # Determine linear and angular velocity
        output = self.get_twist_for_arch(
            angular_error,
        )

        return output

    def get_twist_for_arch(
        self,
        angular_error: float,
    ) -> tuple[Twist, bool]:
        """
        Helper function for Pure Pursuit:
        Returns the angular and linear velocity needed to follow a given arch based on the lookahead_dist
        as well as the error.
        :param angular_error: The error between the rover's dir and the target's dir
        :return: A tuple of the drive command and a boolean indicating whether the rover is at the target position.
        :modifies: self._last_lookahead_dist
        """
        # Obtain parameters
        min_turning_effort = self.node.get_parameter("drive.min_turning_effort").value
        max_turning_effort = self.node.get_parameter("drive.max_turning_effort").value
        min_driving_effort = self.node.get_parameter("drive.min_driving_effort").value
        max_driving_effort = self.node.get_parameter("drive.max_driving_effort").value
        min_lookahead_dist = self.node.get_parameter("pure_pursuit.min_lookahead_distance").value
        max_lookahead_dist = self.node.get_parameter("pure_pursuit.max_lookahead_distance").value

        # Modify the error to be in the range -pi/2 to pi/2
        edited_err = np.clip(angular_error, -1 * np.pi / 2, np.pi / 2)

        # Drive controller for linear and angular velocity
        # Equation for angular velocity was determined through papers such as Purdue's
        # Equation for linear velocity varies based on system, was found through testing and referencing other works
        linear_vel = max_driving_effort * max(0.0001, np.cos(abs(edited_err) + (np.pi / 2)) + 1)
        angular_vel = (2 * np.sin(angular_error) / self._last_lookahead_dist) * 1 / linear_vel

        # Dynamically adjust the lookahead distance based on linear velocity
        self._last_lookahead_dist = (
            (max_lookahead_dist - min_lookahead_dist) * (linear_vel / max_driving_effort)
        ) + min_lookahead_dist

        # Return velocity command
        cmd_vel = Twist(
            linear=Vector3(
                x=np.clip(
                    linear_vel,
                    min_driving_effort,
                    max_driving_effort,
                )
            ),
            angular=Vector3(
                z=np.clip(
                    angular_vel,
                    min_turning_effort,
                    max_turning_effort,
                )
            ),
        )
        return cmd_vel, False

    def determine_next_point(
        self,
        waypoints: Trajectory,
        intersections: list,
    ) -> np.ndarray:
        """
        Helper function for Pure Pursuit:
        Returns the best point for the rover to follow when using Pure Pursuit
        :param waypoints: The current trajectory of the rover.
        :param intersections: The intersections between the rovers lookahead distance and its path (trajectory)
        :return: The current target point that the rover should follow
        :modifies: nothing
        """
        # Protection for if astar traj wasn't used
        if len(waypoints.coordinates != 1):
            waypoints.increment_point()

        # The end is in lookahead dist
        if waypoints.done():
            waypoints.decerement_point()
            if not len(intersections):
                return waypoints.get_current_point()

        # Intersections are empty (get back on to the path)
        if not len(intersections):
            # Get back to the path
            waypoints.decerement_point()
            return waypoints.get_current_point()

        # There is only one intersection
        if len(intersections) == 1:
            waypoints.decerement_point()
            return intersections[0]

        path_point_x = waypoints.get_current_point()[0]
        path_point_y = waypoints.get_current_point()[1]

        dist_1 = np.sqrt((path_point_x - intersections[0][0]) ** 2 + (path_point_y - intersections[0][1]) ** 2)
        dist_2 = np.sqrt((path_point_x - intersections[1][0]) ** 2 + (path_point_y - intersections[1][1]) ** 2)

        target_pos = np.array([0.0, 0.0, 0.0])

        # Pick the intersection point which is closer to the next point in the trajectory
        if dist_1 < dist_2:
            target_pos = np.array([intersections[0][0], intersections[0][1], 0.0])
        else:
            target_pos = np.array([intersections[1][0], intersections[1][1], 0.0])

        # Protection for if astar traj wasn't used
        if len(waypoints.coordinates != 1):
            waypoints.decerement_point()

        return target_pos

    @staticmethod
    def set_farthest_path_point(
        self,
        waypoints: Trajectory,
        rover_pos: np.ndarray,
    ):
        """
        Helper function for Pure Pursuit:
        Adjusts the current trajectory of the rover based on which points are within the
        rovers lookahead distance. Determines the farthest found waypoint in the trajectory
        :param waypoints: The current trajectory of the rover.
        :param rover_pos: The current position of the rover.
        :return: nothing
        :modifies: nothing
        """

        stop_incrementing = False
        next_point: np.ndarray = np.ndarray([])

        # Determine next point
        while not waypoints.done() and not stop_incrementing:
            # Increment to the next point in the path
            waypoints.increment_point()

            if waypoints.done():
                # If we are at the end of the path, we need to determine if this is a point we have seen
                waypoints.decerement_point()
                next_point = waypoints.get_current_point()  # Obtain last point
                waypoints.increment_point()  # Leave the traj to be done

                # Will determine if we continue to go to the last point or we are done
                if self._last_lookahead_dist > np.linalg.norm(next_point - rover_pos):
                    return
                stop_incrementing = True
                break

            # Check if the next point is valid
            next_point = waypoints.get_current_point()
            if self._last_lookahead_dist <= np.linalg.norm(next_point - rover_pos):
                stop_incrementing = True

        if stop_incrementing:
            waypoints.decerement_point()

    @staticmethod
    def sign(val: float) -> float:
        """
        Helper function for compute_intersection_point:
        Gives the sign of a float
        :param val: float
        :return: 1 or -1
        :modifies: nothing
        """

        if val >= 0:
            return 1.0
        else:
            return -1.0

    def compute_intersection_point(self, waypoints: Trajectory, rover_pos: np.ndarray) -> list:
        """
        Helper function for Pure Pursuit:
        Computes the potential points that the rover should follow
        when using pure pursuit
        :param waypoints: The current trajectory of the rover.
        :param rover_pos: The current position of the rover.
        :return: A list of valid intersection points
        :modifies: nothing
        """
        intersections: list = []

        # Ensure a trajectory was passed through
        if waypoints is None:
            raise ValueError("Attempt to detect intersection with no waypoints")
            return

        x1 = waypoints.get_current_point()[0]
        y1 = waypoints.get_current_point()[1]

        # If the goal point is last, return the goal
        if waypoints.increment_point():
            waypoints.decerement_point()

            # astar was never used, just use the segment between the rover and the point
            if len(waypoints.coordinates) == 1:
                x1 = rover_pos[0]
                y1 = rover_pos[1]
            else:
                return intersections

        x_pos = rover_pos[0]
        y_pos = rover_pos[1]
        x2 = waypoints.get_current_point()[0]
        y2 = waypoints.get_current_point()[1]

        # Consider the rovers position as the origin
        x1_offset = x1 - x_pos
        x2_offset = x2 - x_pos
        y1_offset = y1 - y_pos
        y2_offset = y2 - y_pos

        # Get dx and dy
        d_x = x2_offset - x1_offset
        d_y = y2_offset - y1_offset

        d_r = np.sqrt((d_x**2) + (d_y**2))
        det = x1_offset * y2_offset - x2_offset * y1_offset

        # Calculate the discriminate
        discriminate = ((self._last_lookahead_dist**2) * (d_r**2)) - (det**2)

        valid_intersection_1 = False
        valid_intersection_2 = False
        intersection_1 = [0.0, 0.0, 0.0]
        intersection_2 = [0.0, 0.0, 0.0]

        # Determine if there is an intersection
        if discriminate >= 0:
            x_sol_1 = (det * d_y + self.sign(d_y) * d_x * np.sqrt(discriminate)) / (d_r**2)
            x_sol_2 = (det * d_y - self.sign(d_y) * d_x * np.sqrt(discriminate)) / (d_r**2)
            y_sol_1 = (-det * d_x + abs(d_y) * np.sqrt(discriminate)) / (d_r**2)
            y_sol_2 = (-det * d_x - abs(d_y) * np.sqrt(discriminate)) / (d_r**2)

            intersection_1 = [x_sol_1 + x_pos, y_sol_1 + y_pos, 0.0]
            intersection_2 = [x_sol_2 + x_pos, y_sol_2 + y_pos, 0.0]

            min_x = min(x1, x2)
            min_y = min(y1, y2)
            max_x = max(x1, x2)
            max_y = max(y1, y2)

            if min_x <= intersection_1[0] <= max_x and min_y <= intersection_1[1] <= max_y:
                valid_intersection_1 = True
            if min_x <= intersection_2[0] <= max_x and min_y <= intersection_2[1] <= max_y:
                valid_intersection_2 = True
        else:
            # Protection for if astar traj wasn't used
            if len(waypoints.coordinates) != 1:
                waypoints.decerement_point()

            intersections.append([x2, y2, 0.0])
            return intersections

        if valid_intersection_1:
            intersections.append(intersection_1)
        if valid_intersection_2:
            intersections.append(intersection_2)

        # Protection for if astar traj wasn't used
        if len(waypoints.coordinates) != 1:
            waypoints.decerement_point()

        if not len(intersections):
            intersections.append([x2, y2, 0.0])
        return intersections

    def display_markers(
        self,
        rover_pos: np.ndarray,
        intersection_points: np.ndarray,
    ):
        """
        Displays lookahead circle and intersection point for Pure Pursuit
        when using pure pursuit
        :param waypoints: The current trajectory of the rover.
        :param intersection_points: List of potential intersection points
        :return: nothing
        :modifies: nothing
        """

        self.intersection_pub.publish(
            gen_marker(
                time=self.node.get_clock().now(),
                point=[intersection_points[0], intersection_points[1]],
                color=[0.0, 1.0, 0.0],
                id=1,
                lifetime=self.node.get_parameter("pub_path_rate").value,
                size=0.2,
            )
        )

        self.lookahead_pub.publish(
            ring_marker(
                time=self.node.get_clock().now(),
                point=rover_pos,
                color=[1.0, 0.0, 0.0],
                id=1,
                lifetime=self.node.get_parameter("pub_path_rate").value,
                radius=self._last_lookahead_dist,
            )
        )
