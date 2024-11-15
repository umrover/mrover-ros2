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
    traj: Trajectory
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
        self.TRAVERSABLE_COST = context.node.get_parameter("search.traversable_cost").value
        self.time_last_updated = context.node.get_clock().now() - Duration(seconds=self.UPDATE_DELAY)
        self.astar_traj = Trajectory(np.array([]))
        self.traj = Trajectory(np.array([]))
        self.follow_astar = False
        context.env.arrived_at_waypoint = False
            

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
        #context.node.get_logger().info("Waypoint loop")

        assert context.course is not None

        if len(self.traj.coordinates) == 0:
            if context.rover.get_pose_in_map() is None:
                return self
            
            context.node.get_logger().info("Generating segmented path")
            self.segment_path(context)
            return self

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
        

        if not hasattr(context.env.cost_map, 'data'): 
            context.node.get_logger().warn(f"No costmap found, waiting...")
            return self
        
        # If there are no more points in the current a_star path or we are past the update delay, then create a new one
        if len(self.astar_traj.coordinates) == 0 or \
            context.node.get_clock().now() - self.time_last_updated > Duration(seconds=self.UPDATE_DELAY):

            if self.is_high_cost_point(context):
                context.node.get_logger().info(f"High cost segment point, skipping it")
                self.traj.increment_point()
                return self

            # Generate a path
            self.astar_traj = self.astar.generate_trajectory(context, self.traj.get_current_point())

            # Decide whether we follow the astar path to the next point in the spiral
            self.follow_astar = self.astar.use_astar(context=context, star_traj=self.astar_traj, trajectory=self.traj.get_current_point())
            self.time_last_updated = context.node.get_clock().now()

        # Attempt to find the waypoint in the TF tree and drive to it
        arrived = False
        cmd_vel = Twist()
        if not self.USE_COSTMAP or not self.follow_astar:
            waypoint_position_in_map = self.traj.get_current_point()
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

                if self.is_high_cost_point(context):
                    context.node.get_logger().info(f"High cost segment point, skipping it")
                    self.traj.increment_point()
                    return self
    
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
                context.node.get_logger().info(f"Arrived at segment point")
                if self.traj.increment_point():
                    context.env.arrived_at_waypoint = True
                    context.node.get_logger().info("Arrived at waypoint")
                    context.move_costmap()
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
    
    def segment_path(self, context: Context, seg_len: float = 2):
        """
        Segment the path from the rover's current position to the current waypoint into equally spaced points

        Args:
            context (Context): The global context object
            seg_len (float, optional): The length of each segment of the path. Defaults to 2.

        Returns:
            Trajectory: The segmented path
        """
        waypoint_translation = context.course.current_waypoint_pose_in_map().translation()[0:2]
        rover_translation = context.rover.get_pose_in_map().translation()[0:2]

        # Create a numpy array with the rover's current position and the waypoint position
        traj_path = np.array([rover_translation, waypoint_translation])

        # Calculate the number of segments needed for the path
        num_segments: int = int(np.ceil(self.astar.d_calc(waypoint_translation,rover_translation) // seg_len))

        # If there is more than one segment, create the segments
        if num_segments > 0:

            # Calculate the direction vector from the rover to the waypoint
            direction = (waypoint_translation - rover_translation) / num_segments

            # Create the segments by adding the direction vector to the rover's position
            traj_path = np.array([np.round(rover_translation + np.floor(i * direction)) for i in range(0, num_segments)])
            np.append(traj_path, waypoint_translation)

        # Create a Trajectory object from the segmented path
        segmented_trajectory = Trajectory(
            np.hstack((traj_path, np.zeros((traj_path.shape[0], 1))))
        )

        context.node.get_logger().info(f"Segmented path: {segmented_trajectory.coordinates}")
        self.traj = segmented_trajectory

    def is_high_cost_point(self, context: Context) -> bool: 
        cost_map = context.env.cost_map.data
        point_ij = self.astar.cartesian_to_ij(self.traj.get_current_point())
        context.node.get_logger().info(f"{cost_map[int(point_ij[0])][int(point_ij[1])]}")
        return cost_map[int(point_ij[0])][int(point_ij[1])] > self.TRAVERSABLE_COST
    
