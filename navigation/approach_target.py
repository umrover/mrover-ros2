import numpy as np

from state_machine.state import State
from . import search, costmap_search, waypoint, state, recovery
from .context import Context
from navigation.trajectory import Trajectory, SearchTrajectory
from navigation.astar import AStar
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from rclpy.publisher import Publisher
from rclpy.time import Time
from rclpy.duration import Duration
from typing import Any
from navigation.coordinate_utils import gen_marker, is_high_cost_point, d_calc, segment_path


class ApproachTargetState(State):
    astar_traj: Trajectory
    traj: Trajectory
    astar: AStar
    marker_pub: Publisher
    follow_astar: bool
    time_last_updated: Time
    UPDATE_DELAY: float
    USE_COSTMAP: bool
    target_position: np.ndarray | None
    goto_near_point: bool
    near_point: np.ndarray
    cost_inflation_radius: float

    def on_enter(self, context: Context) -> None:
        from .long_range import LongRangeState

        if isinstance(self, LongRangeState):
            context.node.get_logger().info("In long range state")
        else:
            context.node.get_logger().info("In approach target state")
        self.marker_pub = context.node.create_publisher(Marker, "target_trajectory", 10)
        self.time_begin = context.node.get_clock().now()
        self.astar_traj = Trajectory(np.array([]))
        self.traj = Trajectory(np.array([]))
        self.astar = AStar(context=context)
        self.follow_astar = False
        self.goto_near_point = False
        self.near_point = np.array([])
        temp = self.get_target_position(context)
        if temp is not None:
            self.target_position = temp
        else:
            self.target_position = np.array([0, 0])
        self.time_last_updated = context.node.get_clock().now()
        self.UPDATE_DELAY = context.node.get_parameter("search.update_delay").value
        self.USE_COSTMAP = context.node.get_parameter("search.use_costmap").value
        self.DISTANCE_THRESHOLD = context.node.get_parameter("search.distance_threshold").value
        self.cost_inflation_radius = context.node.get_parameter("search.initial_inflation_radius").value

    def on_exit(self, context: Context) -> None:
        pass

    def get_target_position(self, context: Context) -> np.ndarray | None:
        if self.goto_near_point:
            return self.near_point
        return context.env.current_target_pos()

    def next_state(self, context: Context, is_finished: bool) -> State:
        assert context.course is not None
        if is_finished:
            total_time = context.node.get_clock().now() - self.time_begin
            context.course.increment_waypoint()
            context.node.get_logger().info(f"Total approach time: {total_time.nanoseconds // 1000000000}")
            return state.DoneState()

        if context.rover.stuck:
            context.rover.previous_state = self
            return recovery.RecoveryState()

        context.node.get_logger().info("Found closest point to target")
        return self

    def calc_point(self, context: Context, check_astar=False):
        """
        Calculate the nearest low-cost point to the target.
        Should only run if the target is in a high-cost area.

        :param context: Context object providing necessary data.
        :return: True if a low-cost point was found, False otherwise.
        """

        # Get the target position
        target_position = self.target_position

        # Check if the target is in a high-cost area.
        # Now using an explicit check against None.
        if target_position is not None and not is_high_cost_point(point=target_position, context=context) and not check_astar:
            # If not in a high-cost area, return success (no need to find a new point)
            return

        # Define initial search parameters
        search_radius = 1.0  # Adjust based on the scale of your environment
        resolution = 0.1  # Step size for searching in each direction
        max_radius = 10.0  # Maximum allowable radius to expand search
        radius_increment = 1.0  # How much larger to expand radius

        if check_astar:
            resolution = 0.5

        while search_radius <= max_radius:
            # Generate candidate points within the search radius
            candidates = []
            for dx in np.arange(-search_radius, search_radius + resolution, resolution):
                for dy in np.arange(-search_radius, search_radius + resolution, resolution):
                    candidate = target_position + np.array([dx, dy, 0.0])
                    if not is_high_cost_point(point=candidate, context=context):
                        if check_astar:  # If we need to consider A* (astar), generate a trajectory to see if it's reachable
                            candidate_astar_traj = self.astar.generate_trajectory(context, candidate)
                            if len(candidate_astar_traj.coordinates) != 0:
                                candidates.append(candidate)
                        else:
                            candidates.append(candidate)

            if candidates:
                # Find the nearest low-cost point to the target position
                nearest_low_cost_point = min(candidates, key=lambda p: np.linalg.norm(p - target_position))
                self.near_point = nearest_low_cost_point

                # Update internal state (if necessary) or transition to the new point
                self.target_position = nearest_low_cost_point
                self.traj = segment_path(context=context, dest=nearest_low_cost_point[0:2])
                context.node.get_logger().info("Switching target to nearest low-cost point")
                self.astar_traj = Trajectory(np.array([]))
                self.display_markers(context=context)
                self.goto_near_point = True

                context.node.get_logger().info(f"Nearest low cost point: {self.near_point}")

                return

            # Expand the search radius
            search_radius += radius_increment

        # If no low-cost point is found within the maximum radius, raise error
        raise RuntimeError("Couldn't find a better point.")

    def on_loop(self, context: Context) -> State:
        """
        Drive towards a target based on what gets returned from get_target_position().
        Return to search if there is no target position.
        :return: Next state
        """
        from .long_range import LongRangeState

        assert context.course is not None

        if context.env.cost_map is None or not hasattr(context.env.cost_map, "data"):
            return self

        if self.target_position is None:
            # If we lose sight of the target and we have not reached the waypoint yet and are in the long range state,
            # go back to following the waypoint
            if isinstance(self, LongRangeState) and not context.env.arrived_at_waypoint:
                return waypoint.WaypointState()

            # Otherwise, if we lost sight of the target, but were in the regular state it means we were pretty
            # close so we should just return to spiral searching
            return costmap_search.CostmapSearchState()

        # Establish rover's position in the world
        rover_in_map = context.rover.get_pose_in_map()
        if rover_in_map is None:
            return self

        # Wait before starting state
        if context.node.get_clock().now() - Duration(nanoseconds=1000000000) < self.time_begin:
            return self

        if (
            context.node.get_clock().now() - self.time_last_updated > Duration(seconds=self.UPDATE_DELAY)
            or self.target_position is None
        ):
            # Update the time last updated and update the position of the target.
            self.time_last_updated = context.node.get_clock().now()
            self.target_position = self.get_target_position(context)
            if self.target_position is not None:
                context.node.get_logger().info(f"Current target position {self.target_position}")
            else:
                context.node.get_logger().info("No target position, should be transitioning to waypoint/search state soon")
            self.traj = Trajectory(np.array([]))

            # Check if the object is in the ZED, and if so, transition to the regular approach target state
            if isinstance(self, LongRangeState):
                return self.next_state(context, False)

            # Make sure to return self in case get_target_position returned None
            return self

        # Target position must exist
        assert self.target_position is not None

        # If there are no more coordinates left in the trajectory to the target, redevelop the trajectory with segmentation
        if len(self.traj.coordinates) == 0:
            context.node.get_logger().info("Generating approach segmented path")
            self.traj = segment_path(context=context, dest=self.target_position[0:2])
            self.astar_traj = Trajectory(np.array([]))
            self.display_markers(context=context)

        # Check the current point in the trajectory; if it's high cost, move to the next point and check again
        while self.traj.get_current_point() is not None and is_high_cost_point(
            context=context, point=self.traj.get_current_point()
        ):
            context.node.get_logger().info(f"Too high cost point: {self.traj.get_current_point()}")
            if self.traj.increment_point():
                # TODO: What do we do if we skip high cost points and reach the end of the trajectory
                self.calc_point(context=context)
                return self.next_state(context=context, is_finished=False)
            self.display_markers(context=context)
            # Make sure that if going to the nearest point, it's actually in the distance threshold; otherwise, dilate
            if (
                not isinstance(self, LongRangeState)
                and self.goto_near_point
                and not self.point_in_distance_threshold(context, self.get_target_position(context))
            ):
                self.dilate_costmap(context)
                self.calc_point(context)
                return self.next_state(context=context, is_finished=False)

        # If there are no more points in the current A* path or we are past the update delay, then create a new one
        if (
            context.node.get_clock().now() - self.time_last_updated > Duration(seconds=self.UPDATE_DELAY)
            or len(self.astar_traj.coordinates) - self.astar_traj.cur_pt == 0
        ):
            self.time_last_updated = context.node.get_clock().now()
            # Generate A* trajectory between segmented points
            self.astar_traj = self.astar.generate_trajectory(context, self.traj.get_current_point())
            if len(self.astar_traj.coordinates) == 0:
                context.node.get_logger().info("Calculating nearest point that can be astared to:")
                self.calc_point(context, True)

        arrived = False
        cmd_vel = Twist()
        if len(self.astar_traj.coordinates) - self.astar_traj.cur_pt != 0:
            curr_point = self.astar_traj.get_current_point()
            cmd_vel, arrived = context.drive.get_drive_command(
                curr_point,
                rover_in_map,
                context.node.get_parameter("single_tag.stop_threshold").value,
                context.node.get_parameter("waypoint.drive_forward_threshold").value,
            )

        if context.rover.stuck:
            context.rover.previous_state = self
            return recovery.RecoveryState()

        # If we have arrived and reached the end of both the A* trajectory and target trajectory, we have finished
        if arrived:
            if self.astar_traj.increment_point():
                context.node.get_logger().info("Arrived at segment point")
                if self.traj.increment_point():
                    self.goto_near_point = False
                    if self.self_in_distance_threshold(context):
                        return self.next_state(context=context, is_finished=True)
                    else:
                        if isinstance(self, LongRangeState):
                            new_targ_pos = self.get_target_position(context)
                            assert new_targ_pos is not None
                            self.target_position = new_targ_pos
                            self.traj = Trajectory(np.array([]))
                        else:
                            self.dilate_costmap(context=context)
                        return self.next_state(context=context, is_finished=False)
                self.display_markers(context=context)
        else:
            context.rover.send_drive_command(cmd_vel)

        return self

    def display_markers(self, context: Context):
        start_pt = self.traj.cur_pt
        end_pt = (
            self.traj.cur_pt + 7 if self.traj.cur_pt + 7 < len(self.traj.coordinates) else len(self.traj.coordinates)
        )
        for i, coord in enumerate(self.traj.coordinates[start_pt:end_pt]):
            self.marker_pub.publish(gen_marker(context=context, point=coord, color=[1.0, 0.0, 1.0], id=i, lifetime=100))
        self.marker_pub.publish(
            gen_marker(
                context=context, point=self.target_position, color=[1.0, 1.0, 0.0], id=15, lifetime=100, size=0.5
            )
        )

    def self_in_distance_threshold(self, context: Context):
        rover_SE3 = context.rover.get_pose_in_map()
        assert rover_SE3 is not None

        target_pos = context.env.current_target_pos()
        if target_pos is None:
            return False

        rover_translation = rover_SE3.translation()[0:2]
        distance_to_target = d_calc(rover_translation, tuple(target_pos))
        context.node.get_logger().info(f"Distance: {distance_to_target}")
        return distance_to_target < self.DISTANCE_THRESHOLD

    def point_in_distance_threshold(self, context: Context, point):
        assert point is not None

        target_pos = context.env.current_target_pos()
        if target_pos is None:
            return False

        distance = d_calc(point, tuple(target_pos))
        context.node.get_logger().info(f"Distance: {distance}")
        return distance < self.DISTANCE_THRESHOLD

    def dilate_costmap(self, context: Context):
        context.node.get_logger().info("Too far from target! Dilating cost map.")
        self.cost_inflation_radius -= 0.2
        context.dilate_cost(self.cost_inflation_radius)
