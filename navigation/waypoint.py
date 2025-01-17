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
from navigation.coordinate_utils import ij_to_cartesian, cartesian_to_ij, vec_angle, d_calc, gen_marker
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
    traj: Trajectory
    prev_target_pos_in_map: Optional[np.ndarray] = None
    is_recovering: bool = False
    time_last_updated: Time
    start_time = Time
    path_pub: Publisher
    astar: AStar
    follow_astar: bool
    marker_pub: Publisher

    TRAVERSABLE_COST: float
    UPDATE_DELAY: float
    SAFE_APPROACH_DISTANCE: float
    USE_COSTMAP: bool

    def on_enter(self, context: Context) -> None:
        self.marker_pub = context.node.create_publisher(Marker, "spiral_points", 10)
        origin_in_map = context.course.current_waypoint_pose_in_map().translation()[0:2]
        self.astar = AStar(origin_in_map, context)
        self.USE_COSTMAP = context.node.get_parameter("search.use_costmap").value
        self.UPDATE_DELAY = context.node.get_parameter("search.update_delay").value
        self.TRAVERSABLE_COST = context.node.get_parameter("search.traversable_cost").value
        self.time_last_updated = context.node.get_clock().now() - Duration(seconds=self.UPDATE_DELAY)
        self.start_time = context.node.get_clock().now()
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
        
        if len(self.traj.coordinates) == 0:
            if context.rover.get_pose_in_map() is None:
                return self
            
            context.node.get_logger().info("Generating segmented path")
            self.segment_path(context)
            return self

        if not hasattr(context.env.cost_map, 'data'): 
            context.node.get_logger().warn(f"No costmap found, waiting...")
            self.start_time = context.node.get_clock().now()
            return self
        
        
        if context.node.get_clock().now() - Duration(nanoseconds=1000000000) < self.start_time:
            return self
    
        # BEGINNING OF LOGIC

        

        # If there are no more points in the current a_star path or we are past the update delay, then create a new one
        if len(self.astar_traj.coordinates) == 0 or \
            context.node.get_clock().now() - self.time_last_updated > Duration(seconds=self.UPDATE_DELAY):
            self.marker_pub.publish(gen_marker(context=context, point=context.course.current_waypoint_pose_in_map().translation()[0:2], color=[0.0, 0.0, 1.0], size=0.5, id=-1, lifetime=10000))
            while self.is_high_cost_point(context):
                context.node.get_logger().info(f"High cost segment point, skipping it")
                
                if self.traj.increment_point():
                    context.course.increment_waypoint()
                    self.astar_traj = Trajectory(np.array([]))
                    self.traj = Trajectory(np.array([]))
                    return self
        
            # Generate a path
            self.astar_traj = self.astar.generate_trajectory(context, self.traj.get_current_point())

            # Decide whether we follow the astar path to the next point in the spiral
            self.follow_astar = self.astar.use_astar(context=context, star_traj=self.astar_traj, trajectory=self.traj.get_current_point())
            self.time_last_updated = context.node.get_clock().now()

            start_pt = self.traj.cur_pt - 3 if self.traj.cur_pt - 3 >= 0 else self.traj.cur_pt
            end_pt = self.traj.cur_pt + 7 if self.traj.cur_pt + 7 < len(self.traj.coordinates) else len(self.traj.coordinates)
            for i, coord in enumerate(self.traj.coordinates[start_pt:end_pt]):
                self.marker_pub.publish(gen_marker(context=context, point=coord, color=[1.0,0.0,1.0], id=i))
            

        # Attempt to find the waypoint in the TF tree and drive to it
        arrived = False
        cmd_vel = Twist()
        if not self.USE_COSTMAP or not self.follow_astar:
            waypoint_position_in_map = self.traj.get_current_point()
            cmd_vel,arrived = context.drive.get_drive_command(
                    waypoint_position_in_map,
                    rover_in_map,
                    context.node.get_parameter("waypoint.stop_threshold").value,
                    context.node.get_parameter("waypoint.drive_forward_threshold").value,
            )

        else:
            if not hasattr(context.env.cost_map, 'data'): return self

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
                        #search_state = costmap_search.CostmapSearchState()
                        search_state = search.SearchState()
                        return search_state
                    else:
                        # We finished a regular waypoint, go onto the next one
                        context.course.increment_waypoint()
                        self.astar_traj = Trajectory(np.array([]))
                        self.traj = Trajectory(np.array([]))
                        return self
        else:
            context.rover.send_drive_command(cmd_vel)

        return self
    
    def segment_path(self, context: Context, seg_len: float = 1):
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
        num_segments: int = int(np.ceil(d_calc(waypoint_translation,rover_translation) // seg_len))

        # If there is more than one segment, create the segments
        if num_segments > 0:

            # Calculate the direction vector from the rover to the waypoint
            direction = (waypoint_translation - rover_translation) / num_segments

            # Create the segments by adding the direction vector to the rover's position
            traj_path = np.array([rover_translation + i * direction for i in range(0, num_segments)])
            np.append(traj_path, waypoint_translation)

        # Create a Trajectory object from the segmented path
        segmented_trajectory = Trajectory(
            np.hstack((traj_path, np.zeros((traj_path.shape[0], 1))))
        )

        context.node.get_logger().info(f"Segmented path: {segmented_trajectory.coordinates}")
        self.traj = segmented_trajectory

    def is_high_cost_point(self, context: Context) -> bool: 
        cost_map = context.env.cost_map.data
        point_ij = cartesian_to_ij(context, self.traj.get_current_point())

        if not (0 <= int(point_ij[0]) < cost_map.shape[0] and 0 <= int(point_ij[1]) < cost_map.shape[1]):
            context.node.get_logger().warn("Point is out of bounds in the costmap")
            return False

        #context.node.get_logger().info(f"{cost_map[int(point_ij[0])][int(point_ij[1])]}")
        return cost_map[int(point_ij[0])][int(point_ij[1])] > self.TRAVERSABLE_COST
    
    def is_path_blocked(self, context: Context) -> bool:
        if not hasattr(context.env.cost_map, 'data'): return False

        cost_map = context.env.cost_map.data
        rover_pos = context.rover.get_pose_in_map().translation()[0:2]
        target_pos = self.traj.get_current_point()[0:2]
        
        direction = target_pos - rover_pos
        direction_norm = direction / np.linalg.norm(direction)

        CHECK_DISTANCE = 2.0    # meters ahead to check
        CHECK_WIDTH = 2.0       # meters to each side of center
        NUM_WIDTH_CHECKS = 5    # number of points to check across width

        check_center = rover_pos + direction_norm * CHECK_DISTANCE

        perp_vec = np.array([-direction_norm[1], direction_norm[0]])

        for width_mult in np.linspace(-1, 1, NUM_WIDTH_CHECKS):
            check_point = check_center + perp_vec * (width_mult * CHECK_WIDTH)
            
            # Convert to costmap indices
            point_ij = cartesian_to_ij(context, check_point)
            i, j = int(point_ij[0]), int(point_ij[1])
            
            # Boundary check
            if 0 <= i < cost_map.shape[0] and 0 <= j < cost_map.shape[1]:
                # Check if cost is above traversable threshold
                if cost_map[i][j] > self.TRAVERSABLE_COST:
                    context.node.get_logger().info(
                        f"Obstacle detected at distance {CHECK_DISTANCE}m, "
                        f"offset {width_mult * CHECK_WIDTH:.2f}m"
                    )
                    return True
    
        return False

    def __gen_marker__(self, point, id, context: Context):
        """
        Creates and publishes a single spherical marker at the specified (x, y, z) coordinates.

        :param point: A tuple or list containing the (x, y) coordinates of the marker. 
                    The Z coordinate is set to 0.0 by default.
        :param context: The context object providing necessary ROS utilities, 
                        such as the node clock for setting the timestamp.
        :return: A Marker object representing the spherical marker with predefined size and color.
        """
        x = point.copy()[0]
        y = point.copy()[1]
        z = 0.0
        
        marker = Marker()
        marker.lifetime = Duration(seconds=10000).to_msg()
        marker.header = Header(frame_id="map")
        marker.header.stamp = context.node.get_clock().now().to_msg()
        
        marker.ns = "single_point"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set the scale (size) of the sphere
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Set the color (RGBA)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # fully opaque

        # Define the position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        # Orientation is irrelevant for a sphere but must be valid
        marker.pose.orientation.w = 1.0

        return marker
