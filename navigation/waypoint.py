from state_machine.state import State
from . import (
    search,
    recovery,
    post_backup,
    state,
    water_bottle_search,
    costmap_search,
)
from mrover.msg import WaypointType
from mrover.srv import MoveCostMap
from .context import Context
import rclpy
from .context import Context
from navigation.astar import AStar, SpiralEnd, NoPath
from navigation.trajectory import Trajectory, SearchTrajectory
from typing import Optional
from rclpy.publisher import Publisher
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Header
import numpy as np


class WaypointState(State):
    # STOP_THRESHOLD: float = rospy.get_param("waypoint/stop_threshold")
    # DRIVE_FORWARD_THRESHOLD: float = rospy.get_param("waypoint/drive_forward_threshold")
    # USE_COSTMAP: bool = rospy.get_param("water_bottle_search.use_costmap")
    # NO_TAG: int = -1
    astar_traj: Trajectory
    prev_target_pos_in_map: Optional[np.ndarray] = None
    is_recovering: bool = False
    time_last_updated: Time
    path_pub: Publisher
    astar: AStar
    follow_astar: bool

    TRAVERSABLE_COST: float
    UPDATE_DELAY: float
    SAFE_APPROACH_DISTANCE: float
    USE_COSTMAP: bool

    def on_enter(self, context: Context) -> None:
        origin_in_map = context.course.current_waypoint_pose_in_map().translation()[0:2]
        self.astar = AStar(origin_in_map, context)
        self.USE_COSTMAP = context.node.get_parameter("search.use_costmap").value
        self.UPDATE_DELAY = context.node.get_parameter("search.update_delay").value
        self.time_last_updated = context.node.get_clock().now() - Duration(seconds=self.UPDATE_DELAY)
        self.astar_traj = Trajectory(np.array([]))
        self.follow_astar = False
        
        assert context.course is not None

        current_waypoint = context.course.current_waypoint()
        assert current_waypoint is not None

        context.env.arrived_at_waypoint = False

        # TODO(neven): add service to move costmap if going to watter bottle search
        if current_waypoint.type.val == WaypointType.WATER_BOTTLE:
            context.node.get_logger().info("Requesting to move cost map")
            client = context.node.create_client(MoveCostMap, "move_cost_map")
            while not client.wait_for_service(timeout_sec=1.0):
                context.node.get_logger().info("waiting for move_cost_map service...")
            req = MoveCostMap.Request()

            req.course = f"course{context.course.waypoint_index}"
            future = client.call_async(req)
            # TODO(neven): make this actually wait for the service to finish
            # context.node.get_logger().info("called thing")
            # rclpy.spin_until_future_complete(context.node, future)
            # while not future.done():
            #     pass
            # if not future.result():
            #     context.node.get_logger().info("move_cost_map service call failed")
            

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
        

        if not hasattr(context.env.cost_map, 'data'): return self
        # If there are no more points in the current a_star path or we are past the update delay, then create a new one
        if len(self.astar_traj.coordinates) == 0 or \
            context.node.get_clock().now() - self.time_last_updated > Duration(seconds=self.UPDATE_DELAY):

            # Generate a path
            self.astar_traj = self.astar.generate_trajectory(context, context.course.current_waypoint_pose_in_map().translation())
            self.time_last_updated = context.node.get_clock().now()

            # Decide whether we follow the astar path to the next point in the spiral
            self.follow_astar = self.astar.use_astar(context=context, star_traj=self.astar_traj, trajectory=context.course.current_waypoint_pose_in_map().translation())

        # Attempt to find the waypoint in the TF tree and drive to it
        arrived = False
        cmd_vel = Twist()
        if not self.USE_COSTMAP or not self.follow_astar:
            waypoint_position_in_map = context.course.current_waypoint_pose_in_map().translation()
            cmd_vel, arrived = context.drive.get_drive_command(
                    waypoint_position_in_map,
                    rover_in_map,
                    context.node.get_parameter("waypoint.stop_threshold").value,
                    context.node.get_parameter("waypoint.drive_forward_threshold").value,
            )

        else:
            if not hasattr(context.env.cost_map, 'data'): return self
            
            if context.node.get_clock().now() - self.time_last_updated > Duration(seconds=self.UPDATE_DELAY):
                self.astar_traj = self.astar.generate_trajectory(context, context.course.current_waypoint_pose_in_map().translation())
                self.time_last_updated = context.node.get_clock().now()

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
                context.env.arrived_at_waypoint = True
                if context.node.get_parameter("search.use_costmap").value and not current_waypoint.type.val == WaypointType.NO_SEARCH:
                    # We finished a waypoint associated with the water bottle, but we have not seen it yet and are using the costmap to search
                    costmap_search_state = costmap_search.CostmapSearchState()
                    return costmap_search_state
                else:
                    # We finished a regular waypoint, go onto the next one
                    context.course.increment_waypoint()
        else:
            context.rover.send_drive_command(cmd_vel)

        return self
