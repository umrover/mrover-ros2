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

    STOP_THRESH: float
    DRIVE_FWD_THRESH: float
    SPIRAL_COVERAGE_RADIUS: float
    SEGMENTS_PER_ROTATION: int
    DISTANCE_BETWEEN_SPIRALS: float
    TRAVERSABLE_COST: float
    UPDATE_DELAY: float
    SAFE_APPROACH_DISTANCE: float

    def generate_astar_path(self, context: Context):
        rover_position_in_map = context.rover.get_pose_in_map().translation()[0:2]

        # If path to next spiral point has minimal cost per cell, continue normally to next spiral point
        self.star_traj = Trajectory(np.array([]))
        context.node.get_logger().info("Running A*...")
        context.rover.send_drive_command(Twist())  # stop while planning
        try:
            occupancy_list = self.astar.a_star(rover_position_in_map, CostmapSearchState.trajectory.get_current_point())

        except SpiralEnd:
            # TODO: what to do in this case
            CostmapSearchState.trajectory.reset()
            occupancy_list = None

        except NoPath:
            # increment end point
            if CostmapSearchState.trajectory.increment_point():
                # TODO: what to do in this case
                CostmapSearchState.trajectory.reset()
            occupancy_list = None

        if occupancy_list is None:
            self.star_traj = Trajectory(np.array([]))
        else:
            cartesian_coords = self.astar.ij_to_cartesian(np.array(occupancy_list))
            self.star_traj = Trajectory(
                np.hstack((cartesian_coords, np.zeros((cartesian_coords.shape[0], 1))))
            )  # current point gets set back to 0

            # create path type to publish planned path segments to see in rviz
            path = Path()
            poses = []
            path.header = Header()
            path.header.frame_id = "map"
            for coord in cartesian_coords:
                pose_stamped = PoseStamped()
                pose_stamped.header = Header()
                pose_stamped.header.frame_id = "map"
                point = Point(x=coord[0], y=coord[1], z=0.0)
                quat = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                pose_stamped.pose = Pose(position=point, orientation=quat)
                poses.append(pose_stamped)
            path.poses = poses
            self.path_pub.publish(path)

        self.time_last_updated = context.node.get_clock().now()

    def on_enter(self, context: Context) -> None:
        self.STOP_THRESH = context.node.get_parameter("search.stop_threshold").value
        self.DRIVE_FWD_THRESH = context.node.get_parameter("search.drive_forward_threshold").value
        self.SPIRAL_COVERAGE_RADIUS = context.node.get_parameter("search.coverage_radius").value
        self.SEGMENTS_PER_ROTATION = context.node.get_parameter("search.segments_per_rotation").value
        self.DISTANCE_BETWEEN_SPIRALS = context.node.get_parameter("search.distance_between_spirals").value
        self.TRAVERSABLE_COST = context.node.get_parameter("search.traversable_cost").value
        self.UPDATE_DELAY = context.node.get_parameter("search.update_delay").value
        self.SAFE_APPROACH_DISTANCE = context.node.get_parameter("search.safe_approach_distance").value

        if CostmapSearchState.trajectory is None:
            self.new_trajectory(context)

        if not self.is_recovering:
            self.prev_target_pos_in_map = None

        origin_in_map = context.course.current_waypoint_pose_in_map().translation()[0:2]
        self.astar = AStar(origin_in_map, context)
        context.node.get_logger().info(f"Origin: {origin_in_map}")
        self.star_traj = Trajectory(np.array([]))
        self.time_last_updated = context.node.get_clock().now()
        self.path_pub = context.node.create_publisher(Path, "path", 10)

    def on_exit(self, context: Context) -> None:
        pass

    def on_loop(self, context: Context) -> State:
        if not hasattr(context.env.cost_map, 'data'): return self

        assert context.course is not None

        rover_in_map = context.rover.get_pose_in_map()
        assert rover_in_map is not None

        target_object_in_map = context.env.current_target_pos()

        
        # If there are no more points in the current a_star path, then create a new one
        if len(self.star_traj.coordinates) - self.star_traj.cur_pt == 0:
            self.star_traj = Trajectory(np.array([]))
            # If there are no more points in the spiral trajectory, we have finished
            if CostmapSearchState.trajectory.increment_point():
                return waypoint.WaypointState()
            
            # Otherwise generate a path
            context.node.get_logger().info(f"Generating new A-Star path")
            self.generate_astar_path(context)
        
        if context.node.get_clock().now() - self.time_last_updated > Duration(seconds=self.UPDATE_DELAY):
            context.node.get_logger().info(f"Generating new A-Star path")
            self.generate_astar_path(context)
            self.time_last_updated = context.node.get_clock().now()

        # Continue executing the path from wherever it left off
        if len(self.star_traj.coordinates) - self.star_traj.cur_pt != 0: 
            target_position_in_map = self.star_traj.get_current_point()

            cmd_vel, arrived = context.drive.get_drive_command(
                target_position_in_map,
                rover_in_map,
                self.STOP_THRESH,
                self.DRIVE_FWD_THRESH,
                path_start=self.prev_target_pos_in_map,
            )

            if not arrived: context.rover.send_drive_command(cmd_vel)
            else: self.star_traj.increment_point()

            if context.rover.stuck:
                context.rover.previous_state = self
                self.is_recovering = True
                return recovery.RecoveryState()
            else:
                self.is_recovering = False

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

        if (
            target_object_in_map is not None
        ):
            return approach_target.ApproachTargetState()

        return self

    def new_trajectory(self, context) -> None:
        assert context.course is not None
        search_center = context.course.current_waypoint()
        assert search_center is not None

        if not self.is_recovering:
            CostmapSearchState.trajectory = SearchTrajectory.spiral_traj(
                context.course.current_waypoint_pose_in_map().translation()[0:2],
                self.SPIRAL_COVERAGE_RADIUS,
                self.DISTANCE_BETWEEN_SPIRALS,
                self.SEGMENTS_PER_ROTATION,
                search_center.tag_id,
                True,
            )