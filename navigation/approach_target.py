import numpy as np
from typing import Any
from navigation.trajectory import Trajectory
from navigation.astar import AStar, NoPath, OutOfBounds
from . import costmap_search, stuck_recovery, waypoint, backup, state
from .context import Context
from state_machine.state import State
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from rclpy.publisher import Publisher
from rclpy.time import Time
from rclpy.timer import Timer
from rclpy.duration import Duration
from navigation.coordinate_utils import is_high_cost_point, d_calc, segment_path, cartesian_to_ij, ij_to_cartesian, publish_trajectory


class ApproachTargetState(State):
    UPDATE_DELAY: float
    USE_COSTMAP: bool
    USE_PURE_PURSUIT: bool
    DISTANCE_THRESHOLD: float
    COST_INFLATION_RADIUS: float
    time_begin: Time

    astar_traj: Trajectory
    target_traj: Trajectory

    astar: AStar
    marker_pub: Publisher
    time_last_updated: Time
    target_position: np.ndarray | None
    marker_timer: Timer
    update_timer: Timer

    def on_enter(self, context: Context) -> None:
        from .long_range import LongRangeState

        if context.course is None:
            return

        state = "Long Range State" if isinstance(self, LongRangeState) else "Approach Target State"
        context.node.get_logger().info(f"Entered {state}")
        context.rover.previous_state = LongRangeState() if isinstance(self, LongRangeState) else ApproachTargetState()

        self.UPDATE_DELAY = context.node.get_parameter("search.update_delay").value

        current_waypoint = context.course.current_waypoint()

        if current_waypoint is None:
            return

        self.USE_COSTMAP = context.node.get_parameter("costmap.use_costmap").value or current_waypoint.enable_costmap
        self.USE_PURE_PURSUIT = context.node.get_parameter_or("pure_pursuit.use_pure_pursuit", True).value
        self.DISTANCE_THRESHOLD = context.node.get_parameter("search.distance_threshold").value
        self.COST_INFLATION_RADIUS = context.node.get_parameter("costmap.initial_inflation_radius").value
        self.marker_pub = context.node.create_publisher(Marker, "target_trajectory", 10)
        self.astar_traj = Trajectory(np.array([]))
        self.target_traj = Trajectory(np.array([]))
        self.astar = AStar(context=context)
        self.target_position = None
        self.time_last_updated = context.node.get_clock().now()
        self.time_begin = context.node.get_clock().now()

        self.marker_timer = context.node.create_timer(
            context.node.get_parameter("pub_path_rate").value, lambda: self.display_markers(context=context)
        )
        self.update_timer = context.node.create_timer(
            self.UPDATE_DELAY, lambda: self.update_target_position(context=context)
        )

    def on_exit(self, context: Context) -> None:
        self.marker_timer.cancel()
        self.update_timer.cancel()

    def get_target_position(self, context: Context) -> np.ndarray | None:
        return context.env.current_target_pos()

    def next_state(self, context: Context, is_finished: bool) -> State:
        if context.course is None:
            return state.DoneState()
        if is_finished:
            total_time = context.node.get_clock().now() - self.time_begin
            context.node.get_logger().info(f"Total approach time: {total_time.nanoseconds / 10e9} seconds")
            context.course.increment_waypoint()
            context.env.arrived_at_target = True
            return state.DoneState()

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
                        candidates.append(candidate)

            if candidates:
                # Find the nearest low-cost point to the target position
                nearest_low_cost_point = min(candidates, key=lambda p: np.linalg.norm(p - target_position))  # type: ignore

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

    def on_loop_costmap_enabled(self, context: Context) -> State:
        from .long_range import LongRangeState

        if not self.USE_COSTMAP:
            return self

        if not context.dilation_done():
            context.node.get_logger().info("Awaiting dilation future to complete")
            return self

        if context.env.cost_map is None or not hasattr(context.env.cost_map, "data"):
            context.node.get_logger().warn("Costmap is enabled but costmap has no data")
            return self

        rover_pose = context.rover.get_pose_in_map()
        if rover_pose is None:
            context.node.get_logger().warn("Rover has no pose, waiting...")
            context.rover.send_drive_command(Twist())
            return self

        # If the target trajectory is empty, develop a new path to it
        if len(self.target_traj.coordinates) == 0:
            context.node.get_logger().info("Generating approach segmented path")

            # Appease the mypy gods
            if self.target_position is None:
                return self
            self.target_traj = segment_path(context=context, dest=self.target_position[0:2])

        # Check the current point in the trajectory; if it's high cost, move to the next point and check again
        while self.target_traj.get_current_point() is not None and is_high_cost_point(
            context=context, point=self.target_traj.get_current_point()
        ):
            context.node.get_logger().info(f"Skipped high cost point")
            self.target_traj.increment_point()

            if self.target_traj.done():
                break

        if not self.target_traj.done():
            costmap_length = context.env.cost_map.data.shape[0]
            curr_point = cartesian_to_ij(context, self.target_traj.get_current_point())
            if not 0 <= int(curr_point[0]) < costmap_length and 0 <= int(curr_point[1]) < costmap_length:
                context.node.get_logger().warn(
                    "Trajectory point out of the map. Clearing trajectory and trying again..."
                )
                self.target_traj.clear()
                return self

        # If the a-star trajectory is empty and there is a segment to pathfind to, generate a new trajectory there
        if self.astar_traj.empty() and not self.target_traj.done():
            try:
                self.astar_traj = self.astar.generate_trajectory(self.target_traj.get_current_point())

            except Exception as e:
                context.node.get_logger().info(str(e))
                return self

            if self.astar_traj.empty():
                self.target_traj.increment_point()

        if self.target_traj.done():
            self.target_traj.clear()
            context.node.get_logger().info("Target unreachable, calculating nearest low-cost point")
            if self.low_cost_point(context=context):
                # If we are approaching a target and it is not within the distance threshold, dilate the cost map and
                # recalculate for a low-cost point
                if not isinstance(self, LongRangeState) and not self.point_in_distance_threshold(
                    context, self.target_position
                ):
                    # If we fail to find a low-cost point, consider the target unreachable and give up
                    if not context.shrink_dilation():
                        context.node.get_logger().info("Low-cost point not found, giving up")
                        return self.next_state(context=context, is_finished=True)

                    else:
                        context.node.get_logger().info("Found low-cost point")
                        return self

        rover_pos = rover_pose.translation()
        rover_pos[2] = 0
        distance_to_target = np.linalg.norm(self.target_position - rover_pos)

        arrived = False
        cmd_vel = Twist()
        if not self.astar_traj.done():
            curr_point = self.astar_traj.get_current_point()
            cmd_vel, arrived = context.drive.get_drive_command(
                (self.astar_traj if self.USE_PURE_PURSUIT and distance_to_target > 1 else curr_point), # Determine if pure pursuit will be used
                rover_pose,
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

                    # If we are in the long range state and have reached the projected point, get a new point to follow
                    if isinstance(self, LongRangeState):
                        self.target_position = self.get_target_position(context)
                        return self

                    # If we are within the distance threshold of the target we have finished
                    if self.self_in_distance_threshold(context):
                        return self.next_state(context=context, is_finished=True)

                    # Otherwise we need to dilate to get closer
                    else:
                        context.node.get_logger().info("Too far from target, dilating costmap")
                        if not context.shrink_dilation():
                            # Fully dilated and still failed, go to next state
                            return self.next_state(context=context, is_finished=True)
                        return self

        else:
            context.rover.send_drive_command(cmd_vel)

        return self

    def on_loop_costmap_disabled(self, context: Context) -> State:
        from .long_range import LongRangeState

        if self.USE_COSTMAP:
            return self

        if self.target_position is None:
            return self

        arrived = False
        cmd_vel = Twist()
        cmd_vel, arrived = context.drive.get_drive_command(
            self.target_position,
            context.rover.get_pose_in_map(),
            context.node.get_parameter("single_tag.stop_threshold").value,
            context.node.get_parameter("waypoint.drive_forward_threshold").value,
        )

        if arrived:
            if isinstance(self, LongRangeState):
                self.target_position = self.get_target_position(context)
                return self
            else:
                return self.next_state(context=context, is_finished=True)
        else:
            context.rover.send_drive_command(cmd_vel)

        return self

    def update_target_position(self, context: Context):
        # Update the target's position if we still see it, otherwise only "forget" the target position after
        # the expiration duration
        if self.get_target_position(context) is None:
            if context.node.get_clock().now() - self.time_last_updated > Duration(
                seconds=context.node.get_parameter("target_expiration_duration").value
            ):
                self.target_position = None
                context.node.get_logger().info("Lost target position, transitioning states")
                return self
        else:
            self.target_position = self.get_target_position(context)
            context.node.get_logger().info(f"Updated target position to {self.target_position}")
            self.time_last_updated = context.node.get_clock().now()

        # Reset the trajectory for the new target position
        self.target_traj.clear()
        self.astar_traj.clear()

    def on_loop(self, context: Context) -> State:
        from .long_range import LongRangeState

        """
        Drive towards a target based on what gets returned from get_target_position().
        Return to search if there is no target position.
        :return: Next state
        """
        if context.course is None:
            return state.DoneState()

        # Establish rover's position in the world
        rover_in_map = context.rover.get_pose_in_map()
        if rover_in_map is None:
            return self

        if context.rover.stuck:
            return stuck_recovery.StuckRecoveryState()

        if self.target_position is None:
            self.update_target_position(context=context)

        if context.node.get_clock().now() < self.time_begin + Duration(seconds=self.UPDATE_DELAY // 2):
            return self

        # If we see the target in the ZED, transition from long range to approach target
        if isinstance(self, LongRangeState) and context.env.current_target_pos() is not None:
            self.marker_timer.cancel()
            self.update_timer.cancel()
            return ApproachTargetState()

        if self.target_position is None:
            # If we lose sight of the target and we have not reached the waypoint yet and are in the long range state,
            # go back to following the waypoint
            if isinstance(self, LongRangeState) and not context.env.arrived_at_waypoint:
                return waypoint.WaypointState()

            # Otherwise, if we lost sight of the target, but were in the regular state it means we were pretty
            # close so we should just return to spiral searching
            return costmap_search.CostmapSearchState()

        if self.USE_COSTMAP:
            return self.on_loop_costmap_enabled(context=context)
        else:
            return self.on_loop_costmap_disabled(context=context)

    def display_markers(self, context: Context):
        if self.target_position is None:
            return
        if self.USE_COSTMAP:
            context.publish_path_marker(points=self.target_traj.coordinates, color=[1.0, 1.0, 0.0], ns=str(type(self)))

            if not self.astar_traj.is_last() and not self.astar_traj.done():
                context.publish_path_marker(
                    points=self.astar_traj.coordinates[self.astar_traj.cur_pt :],
                    color=[1.0, 0.0, 0.0],
                    ns=str(type(AStar)),
                )
            else:
                context.delete_path_marker(ns=str(type(AStar)))
        else:
            context.publish_path_marker(
                points=np.array([self.target_position]), color=[1.0, 1.0, 0.0], ns=str(type(self))
            )

    def self_in_distance_threshold(self, context: Context):
        rover_SE3 = context.rover.get_pose_in_map()
        if rover_SE3 is None:
            return False

        target_pos = context.env.current_target_pos()
        if target_pos is None:
            return False

        rover_translation = rover_SE3.translation()[0:2]
        distance_to_target = d_calc(rover_translation, tuple(target_pos))
        return distance_to_target < self.DISTANCE_THRESHOLD

    def point_in_distance_threshold(self, context: Context, point):
        if point is None:
            return False
        target_pos = context.env.current_target_pos()
        if target_pos is None:
            return False

        distance = d_calc(point, tuple(target_pos))
        return distance < self.DISTANCE_THRESHOLD
