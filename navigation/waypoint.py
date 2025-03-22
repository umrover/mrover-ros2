from state_machine.state import State
from . import (
    search,
    recovery,
    post_backup,
    state,
    costmap_search,
)
from mrover.msg import WaypointType
from mrover.srv import MoveCostMap
from .context import Context
import rclpy
from .context import Context
from navigation.astar import AStar, SpiralEnd, NoPath
from navigation.coordinate_utils import gen_marker, segment_path, is_high_cost_point
from navigation.trajectory import Trajectory, SearchTrajectory
from typing import Optional
from rclpy.publisher import Publisher
from rclpy.time import Time
import time
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import numpy as np


class WaypointState(State):
    # STOP_THRESHOLD: float = rospy.get_param("waypoint/stop_threshold")
    # DRIVE_FORWARD_THRESHOLD: float = rospy.get_param("waypoint/drive_forward_threshold")
    # USE_COSTMAP: bool = rospy.get_param("water_bottle_search.use_costmap")
    # NO_TAG: int = -1
    astar_traj: Trajectory
    waypoint_traj: Trajectory
    prev_target_pos_in_map: Optional[np.ndarray] = None
    is_recovering: bool = False
    time_last_updated: Time
    start_time = Time
    path_pub: Publisher
    astar: AStar
    marker_pub: Publisher

    UPDATE_DELAY: float

    def on_enter(self, context: Context) -> None:
        assert context.course is not None
        context.node.get_logger().info("In waypoint state")
        self.marker_pub = context.node.create_publisher(Marker, "waypoint_trajectory", 10)
        self.astar = AStar(context)
        self.UPDATE_DELAY = context.node.get_parameter("search.update_delay").value
        self.time_last_updated = context.node.get_clock().now() - Duration(seconds=self.UPDATE_DELAY)
        self.start_time = context.node.get_clock().now()
        self.astar_traj = Trajectory(np.array([]))
        self.waypoint_traj = Trajectory(np.array([]))
        context.env.arrived_at_waypoint = False
        self.marker_pub.publish(
            gen_marker(
                context=context,
                point=context.course.current_waypoint_pose_in_map().translation()[0:2],
                color=[0.0, 0.0, 1.0],
                size=0.5,
                id=-1,
                lifetime=10000,
            )
        )

    def on_exit(self, context: Context) -> None:
        pass

    def on_loop(self, context: Context) -> State:
        """
        Handle driving to a waypoint defined by a linearized cartesian position.
        If the waypoint is associated with a tag id, go into that state early if we see it,
        otherwise wait until we get there to conduct a more thorough search.
        :param context: Context object
        :return:        Next state
        """

        assert context.course is not None

        current_waypoint = context.course.current_waypoint()
        if current_waypoint is None:
            return state.DoneState()

        # If we are at a post currently (from a previous leg), backup to avoid collision
        if context.env.arrived_at_target:
            context.env.arrived_at_target = False
            return post_backup.PostBackupState()

        # Returns either ApproachTargetState, LongRangeState, or None
        approach_state = context.course.get_approach_state()
        if approach_state is not None:
            return approach_state

        rover_in_map = context.rover.get_pose_in_map()
        if rover_in_map is None:
            return self

        self.marker_pub.publish(
            gen_marker(
                context=context,
                point=context.course.current_waypoint_pose_in_map().translation()[0:2],
                color=[0.0, 0.0, 1.0],
                size=0.5,
                id=-1,
                lifetime=10000,
            )
        )

        if len(self.waypoint_traj.coordinates) == 0:
            if context.rover.get_pose_in_map() is None:
                return self

            context.node.get_logger().info("Generating segmented path")
            self.waypoint_traj = segment_path(
                context=context, dest=context.course.current_waypoint_pose_in_map().translation()[0:2]
            )

            self.display_markers(context=context)

            return self

        if not hasattr(context.env.cost_map, "data") and context.node.get_parameter("search.use_costmap").value:
            context.node.get_logger().warn(f"No costmap found, waiting...")
            self.start_time = context.node.get_clock().now()
            return self

        if context.node.get_clock().now() - Duration(nanoseconds=1000000000) < self.start_time:
            return self

        # BEGINNING OF LOGIC

        if is_high_cost_point(context=context, point=self.waypoint_traj.get_current_point()):
            context.rover.send_drive_command(Twist())
            if self.waypoint_traj.increment_point():
                return self.next_state(context=context)
            self.display_markers(context=context)
            self.astar_traj = Trajectory(np.array([]))
            return self

        # If there are no more points in the current a_star path or we are past the update delay, then create a new one
        if (
            context.node.get_clock().now() - self.time_last_updated > Duration(seconds=self.UPDATE_DELAY)
            or len(self.astar_traj.coordinates) - self.astar_traj.cur_pt == 0
        ):
            self.time_last_updated = context.node.get_clock().now()
            # Generate a path
            self.astar_traj = self.astar.generate_trajectory(context, self.waypoint_traj.get_current_point())
            return self

        arrived = False
        cmd_vel = Twist()
        if len(self.astar_traj.coordinates) - self.astar_traj.cur_pt != 0:
            waypoint_position_in_map = self.astar_traj.get_current_point()
            cmd_vel, arrived = context.drive.get_drive_command(
                waypoint_position_in_map,
                rover_in_map,
                context.node.get_parameter("waypoint.stop_threshold").value,
                context.node.get_parameter("waypoint.drive_forward_threshold").value,
            )
        if context.rover.stuck:
            context.rover.previous_state = self
            return recovery.RecoveryState()

        if arrived:
            if self.astar_traj.increment_point():
                self.astar_traj = Trajectory(np.array([]))
                context.node.get_logger().info(f"Arrived at segment point")
                if self.waypoint_traj.increment_point():
                    return self.next_state(context=context)
                self.display_markers(context=context)
        else:
            context.rover.send_drive_command(cmd_vel)

        return self

    def next_state(self, context: Context) -> State:
        assert context.course is not None
        context.env.arrived_at_waypoint = True
        context.node.get_logger().info("Arrived at waypoint")
        context.rover.send_drive_command(Twist())
        current_wp = context.course.current_waypoint()
        assert current_wp is not None
        if not current_wp.type.val == WaypointType.NO_SEARCH:
            if context.node.get_parameter("search.use_costmap").value:
                return costmap_search.CostmapSearchState()
            else:
                return search.SearchState()
        else:
            # We finished a regular waypoint, go onto the next one
            context.course.increment_waypoint()
            self.astar_traj = Trajectory(np.array([]))
            self.waypoint_traj = Trajectory(np.array([]))
            return self

    def display_markers(self, context: Context):
        delete = Marker()
        delete.action = Marker.DELETEALL
        self.marker_pub.publish(delete)
        start_pt = self.waypoint_traj.cur_pt
        end_pt = len(self.waypoint_traj.coordinates)
        for i, coord in enumerate(self.waypoint_traj.coordinates[start_pt:end_pt]):
            self.marker_pub.publish(gen_marker(context=context, point=coord, color=[1.0, 0.0, 1.0], id=i, lifetime=2))
