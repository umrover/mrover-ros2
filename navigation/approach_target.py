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
import numpy as np
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
    target_position: np.ndarray

    def on_enter(self, context: Context) -> None:
        self.marker_pub = context.node.create_publisher(Marker, "spiral_points", 10)
        self.time_begin = context.node.get_clock().now()
        self.astar_traj = Trajectory(np.array([]))
        self.traj = Trajectory(np.array([]))
        self.astar = AStar(context=context)
        self.follow_astar = False
        self.target_position = self.get_target_position(context) or np.array([0, 0])
        self.time_last_updated = context.node.get_clock().now()
        self.UPDATE_DELAY = context.node.get_parameter("search.update_delay").value
        self.USE_COSTMAP = context.node.get_parameter("search.use_costmap").value
        pass

    def on_exit(self, context: Context) -> None:
        pass

    def get_target_position(self, context: Context) -> np.ndarray | None:
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

        return self

    def on_loop(self, context: Context) -> State:
        """
        Drive towards a target based on what gets returned from get_target_position().
        Return to search if there is no target position.
        :return: Next state
        """

        assert context.course is not None

        if self.target_position is None:
            from .long_range import LongRangeState

            if isinstance(self, LongRangeState) and not context.env.arrived_at_waypoint:
                return waypoint.WaypointState()

            return costmap_search.CostmapSearchState()

        rover_in_map = context.rover.get_pose_in_map()
        assert rover_in_map is not None

        if not hasattr(context.env.cost_map, "data"):
            context.node.get_logger().warn(f"No costmap found, waiting...")
            self.time_begin = context.node.get_clock().now()
            return self

        if context.node.get_clock().now() - Duration(nanoseconds=1000000000) < self.time_begin:
            return self

        target_pos = self.get_target_position(context)
        assert target_pos is not None
        if len(self.traj.coordinates) == 0 or d_calc(tuple(self.target_position), tuple(target_pos)) > 0.3:
            self.target_position = target_pos
            context.node.get_logger().info("Generating approach segmented path")
            self.traj = segment_path(context=context, dest=self.target_position[0:2])
            self.astar_traj = Trajectory(np.array([]))
            self.display_markers(context=context)

        while self.traj.get_current_point() is not None and is_high_cost_point(
            context=context, point=self.traj.get_current_point()
        ):
            if self.traj.increment_point():
                # TODO: What do we do if we skip high cost points and reach the end of the trajectory
                return self.next_state(context=context, is_finished=False)
            self.display_markers(context=context)

        # If there are no more points in the current a_star path or we are past the update delay, then create a new one
        if (
            context.node.get_clock().now() - self.time_last_updated > Duration(seconds=self.UPDATE_DELAY)
            or len(self.astar_traj.coordinates) - self.astar_traj.cur_pt == 0
        ):
            self.time_last_updated = context.node.get_clock().now()
            # Generate a path
            self.astar_traj = self.astar.generate_trajectory(context, self.traj.get_current_point())

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

        if arrived:
            if self.astar_traj.increment_point():
                context.node.get_logger().info(f"Arrived at segment point")
                if self.traj.increment_point():
                    return self.next_state(context=context, is_finished=True)
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
