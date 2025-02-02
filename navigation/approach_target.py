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
    target_position: np.ndarray | None

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
        self.target_position = self.get_target_position(context)
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
        from .long_range import LongRangeState
        assert context.course is not None

        if self.target_position is None:
            
            # If we lose sight of the target and we have not reached the waypoint yet are we are in the long range state,
            # go back to following the waypoint 
            if isinstance(self, LongRangeState) and not context.env.arrived_at_waypoint:
                return waypoint.WaypointState()

            # Otherwise, if we lost sight of the target, but were in the regular state it means we were pretty 
            # close so we should just return to spiral searching
            return costmap_search.CostmapSearchState()


        # Establish rover's position in the world
        rover_in_map = context.rover.get_pose_in_map()
        assert rover_in_map is not None

        # Assert costmap exists
        if not hasattr(context.env.cost_map, "data"):
            context.node.get_logger().warn(f"No costmap found, waiting...")
            self.time_begin = context.node.get_clock().now()
            return self

        # Wait before starting state
        if context.node.get_clock().now() - Duration(nanoseconds=1000000000) < self.time_begin:
            return self


        if context.node.get_clock().now() - self.time_last_updated > Duration(seconds=self.UPDATE_DELAY) or self.target_position is None:
            
            # Update the time last updated and update the position of the target. Clear the current target trajectory
            self.time_last_updated = context.node.get_clock().now()
            self.target_position = self.get_target_position(context)
            if self.target_position is not None and self.target_position.all(): 
                context.node.get_logger().info(f"Current target position {self.target_position}")
            else: 
                context.node.get_logger().info(f"No target position, should be transitioning to waypoint/search state soon")
            self.traj = Trajectory(np.array([]))

            # Check if the object is in the ZED, and if so, transition to the regular approach target state
            if isinstance(self, LongRangeState):
                return self.next_state(context, False)

            # Make sure to return self in case get_target_position returned None
            return self


        # Target position must exist
        assert self.target_position is not None

        # If there are no more coordinates left in the trajectory to the target redevelop the trajectory with segmentation
        if len(self.traj.coordinates) == 0:
            context.node.get_logger().info("Generating approach segmented path")
            self.traj = segment_path(context=context, dest=self.target_position[0:2])
            self.astar_traj = Trajectory(np.array([]))
            self.display_markers(context=context)


        # Check the current point in the trajectory, if its high cost, let the current point be the next point in the trajectory
        # and check again
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
            # Generate astar trajectory between segmented points
            self.astar_traj = self.astar.generate_trajectory(context, self.traj.get_current_point())


        # Create the twst and check if we have arrived
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

        # If we have arrived and reached the end of both the astar trajectory and target trajectory, we have finished
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
            self.marker_pub.publish(gen_marker(context=context, point=coord, color=[1.0, 0.0, 1.0], id=i, lifetime=2))
