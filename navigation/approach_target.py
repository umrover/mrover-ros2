import numpy as np
from typing import Any
from navigation.trajectory import Trajectory
from navigation.astar import AStar, NoPath, OutOfBounds
from . import costmap_search, waypoint, state, recovery
from .context import Context
from state_machine.state import State
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from rclpy.publisher import Publisher
from rclpy.time import Time
from rclpy.duration import Duration
from navigation.coordinate_utils import gen_marker, is_high_cost_point, d_calc, segment_path


class ApproachTargetState(State):
    UPDATE_DELAY: float
    USE_COSTMAP: bool
    DISTANCE_THRESHOLD: float
    COST_INFLATION_RADIUS: float
    time_begin: Time
    astar_traj: Trajectory
    target_traj: Trajectory
    astar: AStar
    marker_pub: Publisher
    time_last_updated: Time
    target_position: np.ndarray | None

    def on_enter(self, context: Context) -> None:
        from .long_range import LongRangeState

        state = "Long Range State" if isinstance(self, LongRangeState) else "Approach Target State"
        context.node.get_logger().info(f"Entered {state}")

        self.UPDATE_DELAY = context.node.get_parameter("search.update_delay").value
        self.USE_COSTMAP = context.node.get_parameter("costmap.use_costmap").value
        self.DISTANCE_THRESHOLD = context.node.get_parameter("search.distance_threshold").value
        self.COST_INFLATION_RADIUS = context.node.get_parameter("search.initial_inflation_radius").value
        self.marker_pub = context.node.create_publisher(Marker, "target_trajectory", 10)
        self.astar_traj = Trajectory(np.array([]))
        self.target_traj = Trajectory(np.array([]))
        self.astar = AStar(context=context)
        self.target_position = self.get_target_position(context)
        self.time_last_updated = context.node.get_clock().now()
        self.time_begin = context.node.get_clock().now()

    def on_exit(self, context: Context) -> None:
        pass

    def get_target_position(self, context: Context) -> np.ndarray | None:
        return context.env.current_target_pos()

    def next_state(self, context: Context, is_finished: bool) -> State:
        assert context.course is not None
        if is_finished:
            total_time = context.node.get_clock().now() - self.time_begin
            context.node.get_logger().info(f"Total approach time: {total_time.nanoseconds // 1000000000}")
            self.display_markers(context=context)
            if context.course.increment_waypoint():
                return state.DoneState()
            return waypoint.WaypointState()

        if context.rover.stuck:
            context.rover.previous_state = self
            return recovery.RecoveryState()

        context.node.get_logger().info("Found closest point to target")
        return self

    def low_cost_point(self, context: Context, check_astar=False) -> bool:
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
        if (
            target_position is not None
            and not is_high_cost_point(point=target_position, context=context)
            and not check_astar
        ):
            # If not in a high-cost area, return success (no need to find a new point)
            return True

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
                        if (
                            check_astar
                        ):  # If we need to consider A* (astar), generate a trajectory to see if it's reachable
                            candidate_astar_traj = self.astar.generate_trajectory(context, candidate)
                            if len(candidate_astar_traj.coordinates) != 0:
                                candidates.append(candidate)
                        else:
                            candidates.append(candidate)

            if candidates:
                # Find the nearest low-cost point to the target position
                nearest_low_cost_point = min(candidates, key=lambda p: np.linalg.norm(p - target_position))

                # Update internal state (if necessary) or transition to the new point
                self.target_position = nearest_low_cost_point
                self.target_traj.clear()
                self.astar_traj.clear()
                return True

            # Expand the search radius
            search_radius += radius_increment

        # If no low-cost point is found within the maximum radius, raise error
        context.node.get_logger().warn("Could not find a low-cost point within radius")
        return False

    def on_loop(self, context: Context) -> State:
        from .long_range import LongRangeState

        """
        Drive towards a target based on what gets returned from get_target_position().
        Return to search if there is no target position.
        :return: Next state
        """
        assert context.course is not None

        if (
            context.env.cost_map is None
            or not hasattr(context.env.cost_map, "data")
            and context.node.get_parameter("costmap.use_costmap").value
        ):
            return self

        if context.move_costmap_future and not context.move_costmap_future.done():
            return self

        # Establish rover's position in the world
        rover_in_map = context.rover.get_pose_in_map()
        if rover_in_map is None:
            return self

        if context.rover.stuck:
            context.rover.previous_state = self
            return recovery.RecoveryState()

        # If we see the target in the ZED, transition from long range to approach target
        if isinstance(self, LongRangeState) and context.env.current_target_pos() is not None:
            return ApproachTargetState()

        if self.target_position is None:
            # If we lose sight of the target and we have not reached the waypoint yet and are in the long range state,
            # go back to following the waypoint
            if isinstance(self, LongRangeState) and not context.env.arrived_at_waypoint:
                return waypoint.WaypointState()

            # Otherwise, if we lost sight of the target, but were in the regular state it means we were pretty
            # close so we should just return to spiral searching
            return costmap_search.CostmapSearchState()

        # Logic for every update
        if context.node.get_clock().now() - self.time_last_updated > Duration(seconds=self.UPDATE_DELAY):
            # Update the target's position if we still see it, otherwise only "forget" the target position after
            # the expiration duration
            if self.get_target_position(context) is None:
                if (
                    context.node.get_clock().now() - self.time_last_updated
                    > Duration(seconds=self.UPDATE_DELAY)
                    > Duration(seconds=context.node.get_parameter("target_expiration_duration").value)
                ):
                    self.target_position = None
                    context.node.get_logger().info("Lost target position, transitioning states")
                    return self
            else:
                self.target_position = self.get_target_position(context)
                context.node.get_logger().info(f"Updated target position to {self.target_position}")

            # Reset the trajectory for the new target position
            self.target_traj.clear()
            self.astar_traj.clear()

            self.time_last_updated = context.node.get_clock().now()

        # If the target trajectory is empty, develop a new path to it and display its markers
        if len(self.target_traj.coordinates) == 0:
            context.node.get_logger().info("Generating approach segmented path")

            # Appease the mypy gods
            assert self.target_position is not None
            self.target_traj = segment_path(context=context, dest=self.target_position[0:2])

            self.display_markers(context=context)

        # Check the current point in the trajectory; if it's high cost, move to the next point and check again
        while self.target_traj.get_current_point() is not None and is_high_cost_point(
            context=context, point=self.target_traj.get_current_point()
        ):
            context.node.get_logger().info(f"Skipped high cost point")
            self.target_traj.increment_point()

            if self.target_traj.done():
                break

        # If the a-star trajectory is empty and there is a segment to pathfind to, generate a new trajectory there
        if self.astar_traj.empty() and not self.target_traj.done():
            try:
                self.astar_traj = self.astar.generate_trajectory(context, self.target_traj.get_current_point())
            except OutOfBounds:
                context.node.get_logger().warn(
                    "Attempted to generate a trajectory for the rover when it was out of bounds of the costmap"
                )
                self.target_traj.clear()
                return self
            if self.astar_traj.empty():
                self.target_traj.increment_point()

        if self.target_traj.done():
            self.target_traj.clear()
            context.node.get_logger().info("Target unreachable, calculating nearest low-cost point")
            if self.low_cost_point(context=context):
                # If we are approaching a target and it is not within the distance threshold, dilate the cost map and
                # recalculate for a low-cost point
                while not isinstance(self, LongRangeState) and not self.point_in_distance_threshold(
                    context, self.target_position
                ):
                    self.dilate_costmap(context)
                    self.low_cost_point(context)
                context.node.get_logger().info("Found low-cost point")
                return self

            # If we fail to find a low-cost point, consider the target unreachable and give up
            context.node.get_logger().info("Low-cost point not found, giving up")
            return self.next_state(context=context, is_finished=True)

        arrived = False
        cmd_vel = Twist()
        if not self.astar_traj.done():
            curr_point = self.astar_traj.get_current_point()
            cmd_vel, arrived = context.drive.get_drive_command(
                curr_point,
                rover_in_map,
                context.node.get_parameter("single_tag.stop_threshold").value,
                context.node.get_parameter("waypoint.drive_forward_threshold").value,
            )

        # If we have arrived increment the a-star trajectory
        if arrived:
            self.astar_traj.increment_point()

            # If we finished an astar trajectory, increment the target_trajectory
            if self.astar_traj.done():
                self.astar_traj.clear()
                context.node.get_logger().info("Arrived at segment point")
                self.target_traj.increment_point()

                # If we finished the target trajectory
                if self.target_traj.done():
                    self.target_traj.clear()
                    # If we are within the distance threshold of the target we have finished
                    if self.self_in_distance_threshold(context):
                        return self.next_state(context=context, is_finished=True)

                    # Otherwise we need to dilate to get closer
                    else:
                        if isinstance(self, LongRangeState):
                            self.target_position = self.get_target_position(context)
                        else:
                            context.node.get_logger().info("Too far from target, dilating costmap")
                            if not self.dilate_costmap(context=context):
                                # Fully dilated and still failed, go to next state
                                return self.next_state(context=context, is_finished=True)
                        return self.next_state(context=context, is_finished=False)
                self.display_markers(context=context)
        else:
            context.rover.send_drive_command(cmd_vel)

        return self

    def display_markers(self, context: Context):
        if context.node.get_parameter("display_markers").value:
            delete = Marker()
            delete.action = Marker.DELETEALL
            self.marker_pub.publish(delete)
            start_pt = self.target_traj.cur_pt - 2 if self.target_traj.cur_pt - 2 >= 0 else 0
            end_pt = (
                self.target_traj.cur_pt + 7
                if self.target_traj.cur_pt + 7 < len(self.target_traj.coordinates)
                else len(self.target_traj.coordinates)
            )
            for i, coord in enumerate(self.target_traj.coordinates[start_pt:end_pt]):
                self.marker_pub.publish(
                    gen_marker(context=context, point=coord, color=[1.0, 0.0, 1.0], id=i, lifetime=100)
                )
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
        return distance_to_target < self.DISTANCE_THRESHOLD

    def point_in_distance_threshold(self, context: Context, point):
        assert point is not None

        target_pos = context.env.current_target_pos()
        if target_pos is None:
            return False

        distance = d_calc(point, tuple(target_pos))
        return distance < self.DISTANCE_THRESHOLD

    def dilate_costmap(self, context: Context) -> bool:
        self.COST_INFLATION_RADIUS -= 0.2
        context.dilate_cost(self.COST_INFLATION_RADIUS)
        return self.COST_INFLATION_RADIUS > 0
