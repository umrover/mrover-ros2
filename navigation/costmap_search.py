from typing import Optional

import numpy as np

import rclpy
from rclpy.publisher import Publisher
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from mrover.msg import GPSPointList
from nav_msgs.msg import Path
from navigation import approach_target, recovery, waypoint
from navigation.astar import AStar, SpiralEnd, NoPath
from navigation.context import convert_cartesian_to_gps, Context
from navigation.trajectory import Trajectory, SearchTrajectory
from std_msgs.msg import Header
from state_machine.state import State

# REFERENCE: https://docs.google.com/document/d/18GjDWxIu5f5-N5t5UgbrZGdEyaDj9ZMEUuXex8-NKrA/edit
class CostmapSearchState(State):
    """
    General search state that utilizes the costmap for searching for any obstacles
    Follows a search spiral but uses A* to avoid obstacles
    Intended to be implemented over the current search state
    """

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

    def use_astar(self, context: Context) -> bool:
        rover_in_map = context.rover.get_pose_in_map()
        follow_astar: bool
        if rover_in_map is None or \
            self.trajectory.get_current_point() is None or \
            len(self.star_traj.coordinates) < 4: 
            follow_astar = False

        else: 
            astar_dist = 0.0
            for i, coordinate in enumerate(self.star_traj.coordinates[:-1]):
                astar_dist += self.astar.d_calc(self.star_traj.coordinates[i], self.star_traj.coordinates[i+1])

            eucl_dist = self.astar.d_calc(context.rover.get_pose_in_map().translation()[0:2], tuple(self.trajectory.get_current_point()))

            follow_astar = abs(astar_dist - eucl_dist) / eucl_dist > self.A_STAR_THRESH

        if (follow_astar and not self.follow_astar) or (not follow_astar and self.follow_astar):   
            if follow_astar: 
                context.node.get_logger().info(f"Switching to astar path")
            else:
                context.node.get_logger().info(f"Switching to regular path")
        return follow_astar

    def on_enter(self, context: Context) -> None:

        # Parameter initialization
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
        self.time_begin = context.node.get_clock().now()

    def on_exit(self, context: Context) -> None:
        pass

    def on_loop(self, context: Context) -> State:
        # Wait until the costmap is ready
        if not hasattr(context.env.cost_map, 'data'): 
            context.node.get_logger().warn(f"No costmap found, waiting...")
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

        # If there are no more points in the current a_star path or we are past the update delay, then create a new one
        if len(self.star_traj.coordinates) - self.star_traj.cur_pt == 0 or \
            context.node.get_clock().now() - self.time_last_updated > Duration(seconds=self.UPDATE_DELAY):

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
            and self.astar.d_calc(context.env.current_target_pos(), rover_in_map.translation()[0:2]) < self.SAFE_APPROACH_DISTANCE):
            total_time = context.node.get_clock().now() - self.time_begin
            context.node.get_logger().info(f"Total search time: {total_time.nanoseconds/1000000000}")
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