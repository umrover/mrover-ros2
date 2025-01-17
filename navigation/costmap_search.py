from typing import Optional
import numpy as np

import rclpy
from rclpy.publisher import Publisher
from rclpy.time import Time
from rclpy.duration import Duration
import time
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from mrover.msg import GPSPointList
from nav_msgs.msg import Path
from navigation import approach_target, recovery, waypoint
from navigation.astar import AStar, SpiralEnd, NoPath
from navigation.coordinate_utils import d_calc
from navigation.context import convert_cartesian_to_gps, Context
from navigation.trajectory import Trajectory, SearchTrajectory
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from state_machine.state import State
from rclpy.publisher import Publisher

# REFERENCE: https://docs.google.com/document/d/18GjDWxIu5f5-N5t5UgbrZGdEyaDj9ZMEUuXex8-NKrA/edit
class CostmapSearchState(State):
    """
    General search state that utilizes the costmap for searching for any obstacles
    Follows a search spiral but uses A* to avoid obstacles
    Intended to be implemented over the current search state
    """

    prev_pos: Optional[np.ndarray]
    time_begin: Optional[Time]
    total_distance: float

    trajectory: Optional[SearchTrajectory] = None  # spiral
    star_traj: Trajectory  # returned by astar
    prev_target_pos_in_map: Optional[np.ndarray] = None
    is_recovering: bool = False
    time_last_updated: Time
    path_pub: Publisher
    astar: AStar
    follow_astar: bool

    STOP_THRESH: float
    DRIVE_FWD_THRESH: float
    SPIRAL_COVERAGE_RADIUS: float
    SEGMENTS_PER_ROTATION: int
    DISTANCE_BETWEEN_SPIRALS: float
    TRAVERSABLE_COST: float
    UPDATE_DELAY: float
    SAFE_APPROACH_DISTANCE: float
    A_STAR_THRESH: float
    marker_pub: Publisher

    def on_enter(self, context: Context) -> None:

        # Parameter initialization
        self.prev_pos = None
        self.time_begin = None
        self.total_distance = 0.0
        self.marker_pub = context.node.create_publisher(Marker, "spiral_points", 10)
        self.STOP_THRESH = context.node.get_parameter("search.stop_threshold").value
        self.DRIVE_FWD_THRESH = context.node.get_parameter("search.drive_forward_threshold").value
        self.SPIRAL_COVERAGE_RADIUS = context.node.get_parameter("search.coverage_radius").value
        self.SEGMENTS_PER_ROTATION = context.node.get_parameter("search.segments_per_rotation").value
        self.DISTANCE_BETWEEN_SPIRALS = context.node.get_parameter("search.distance_between_spirals").value
        self.TRAVERSABLE_COST = context.node.get_parameter("search.traversable_cost").value
        self.UPDATE_DELAY = context.node.get_parameter("search.update_delay").value
        self.SAFE_APPROACH_DISTANCE = context.node.get_parameter("search.safe_approach_distance").value
        self.A_STAR_THRESH= context.node.get_parameter("search.a_star_thresh").value

        # Creates spiral trajectory to follow
        if self.trajectory is None:
            self.new_trajectory(context)

        if not self.is_recovering:
            self.prev_target_pos_in_map = None

        # Gets a-star ready
        origin_in_map = context.course.current_waypoint_pose_in_map().translation()[0:2]
        self.astar = AStar(origin_in_map, context)
        context.node.get_logger().info(f"Origin: {origin_in_map}")
        self.star_traj = Trajectory(np.array([]))
        self.time_last_updated = context.node.get_clock().now()
        self.follow_astar = False

        #Initialize stopwatch
        self.time_begin = None

    def on_exit(self, context: Context) -> None:
        pass

    def on_loop(self, context: Context) -> State:
        # Wait until the costmap is ready
        if not hasattr(context.env.cost_map, 'data'): 
            context.node.get_logger().warn(f"No costmap found, waiting...")
            time.sleep(1.0)
            return self

        assert context.course is not None

        rover_in_map = context.rover.get_pose_in_map()
        assert rover_in_map is not None
        
        if context.rover.stuck:
            context.rover.previous_state = self
            self.is_recovering = True
            return recovery.RecoveryState()
        else:
            self.is_recovering = False

        if not self.time_begin:
            self.time_begin = context.node.get_clock().now()
            self.prev_pos = rover_in_map.translation()[0:2]
        else:
            self.total_distance += d_calc(rover_in_map.translation()[0:2], self.prev_pos)
            self.prev_pos = rover_in_map.translation()[0:2]

        # If there are no more points in the current a_star path or we are past the update delay, then create a new one
        if len(self.star_traj.coordinates) - self.star_traj.cur_pt == 0 or \
            context.node.get_clock().now() - self.time_last_updated > Duration(seconds=self.UPDATE_DELAY):

            total_time = (context.node.get_clock().now() - self.time_begin).nanoseconds / 1e9
            context.node.get_logger().info(f"Total Distance Traveled: {self.total_distance}m\nTotal Time: {total_time}s\nAverage Speed: {self.total_distance/total_time}m/s")

            start_pt = self.trajectory.cur_pt - 3 if self.trajectory.cur_pt - 3 >= 0 else self.trajectory.cur_pt
            end_pt = self.trajectory.cur_pt + 3 if self.trajectory.cur_pt + 3 < len(self.trajectory.coordinates) else len(self.trajectory.coordinates)
            for i, coord in enumerate(self.trajectory.coordinates[start_pt:end_pt]):
                self.marker_pub.publish(self.__gen_marker__(coord, i, context))

            # If there are no more points in the spiral trajectory, move to the next spiral point
            if len(self.star_traj.coordinates) - self.star_traj.cur_pt == 0:
                self.star_traj = Trajectory(np.array([]))
                if self.trajectory.increment_point():
                    context.node.get_logger().info(f"Reached the end of search spiral")
                    return waypoint.WaypointState()
            
            # Otherwise generate a path
            self.star_traj = self.astar.generate_trajectory(context, self.trajectory.get_current_point())
            self.time_last_updated = context.node.get_clock().now()

            # Decide whether we follow the astar path to the next point in the spiral
            self.follow_astar = self.astar.use_astar(context=context, star_traj=self.star_traj, trajectory=self.trajectory.get_current_point())
        
        # Choosing which path to take: A-Star or regular
        if self.follow_astar:
            # Follow the rest of the astar trajectory from wherever it left off
            if len(self.star_traj.coordinates) - self.star_traj.cur_pt != 0: 
                target_position_in_map = self.star_traj.get_current_point()

                cmd_vel, arrived = context.drive.get_drive_command(
                    target_position_in_map,
                    rover_in_map,
                    self.STOP_THRESH,
                    self.DRIVE_FWD_THRESH,
                    path_start=self.prev_target_pos_in_map,
                )

                if not arrived: 
                    context.rover.send_drive_command(cmd_vel)
                else: 
                    self.prev_target_pos_in_map = target_position_in_map
                    self.star_traj.increment_point()

        else:
            # One drive command to the next spiral point
            target_position_in_map = self.trajectory.get_current_point()
            cmd_vel, arrived = context.drive.get_drive_command(
                    self.trajectory.get_current_point(),
                    rover_in_map,
                    self.STOP_THRESH,
                    self.DRIVE_FWD_THRESH,
                    path_start=self.prev_target_pos_in_map,
                )
            if not arrived: 
                    context.rover.send_drive_command(cmd_vel)
            else:
                self.prev_target_pos_in_map = target_position_in_map

                if self.trajectory.increment_point():
                    return waypoint.WaypointState()
                
                # Otherwise generate a path
                context.node.get_logger().info(f"Reached spiral node. Generating new A-Star path")
                self.star_traj = self.astar.generate_trajectory(context, self.trajectory.get_current_point())
                self.time_last_updated = context.node.get_clock().now()

                # Decide whether we follow the astar path to the next point in the spiral
                self.follow_astar = self.astar.use_astar(context=context, star_traj=self.star_traj, trajectory=self.trajectory.get_current_point())

        # If our target object has been detected, approach it
        if (context.env.current_target_pos() is not None
            and d_calc(context.env.current_target_pos(), rover_in_map.translation()[0:2]) < self.SAFE_APPROACH_DISTANCE):
            total_time = context.node.get_clock().now() - self.time_begin
            context.node.get_logger().info(f"Total search time: {total_time.nanoseconds // 1000000000}")
            return approach_target.ApproachTargetState()

        return self
        
    
        # TODO: Figure out functionality
        # ref = np.array(
        #     [
        #         context.node.get_parameter("ref_lat").value,
        #         context.node.get_parameter("ref_lon").value,
        #         context.node.get_parameter("ref_alt").value,
        #     ]
        # )
        # context.search_point_publisher.publish(
        #     GPSPointList(points=[convert_cartesian_to_gps(ref, pt) for pt in CostmapSearchState.trajectory.coordinates])
        # )

    def new_trajectory(self, context) -> None:
        assert context.course is not None
        search_center = context.course.current_waypoint()
        assert search_center is not None

        if not self.is_recovering:
            self.trajectory = SearchTrajectory.spiral_traj(
                context.course.current_waypoint_pose_in_map().translation()[0:2],
                self.SPIRAL_COVERAGE_RADIUS,
                self.DISTANCE_BETWEEN_SPIRALS,
                self.SEGMENTS_PER_ROTATION,
                search_center.tag_id,
                True,
            )
    
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
        marker.lifetime = Duration(seconds=5).to_msg()
        marker.header = Header(frame_id="map")
        marker.header.stamp = context.node.get_clock().now().to_msg()
        
        marker.ns = "single_point"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set the scale (size) of the sphere
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Set the color (RGBA)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # fully opaque

        # Define the position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        # Orientation is irrelevant for a sphere but must be valid
        marker.pose.orientation.w = 1.0

        return marker