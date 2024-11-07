import numpy as np

from state_machine.state import State
from . import search, waypoint, state, recovery
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


class ApproachTargetState(State):
    trajectory: Trajectory
    prev_target_pos_in_map: Optional[np.ndarray] = None
    is_recovering: bool = False
    time_last_updated: Time
    path_pub: Publisher
    astar: AStar

    SEGMENTS_PER_ROTATION: int
    DISTANCE_BETWEEN_SPIRALS: float
    TRAVERSABLE_COST: float
    UPDATE_DELAY: float
    SAFE_APPROACH_DISTANCE: float
    USE_COSTMAP: bool
    
    def generate_astar_path(self, context: Context):
        rover_position_in_map = context.rover.get_pose_in_map().translation()[0:2]

        # If path to next spiral point has minimal cost per cell, continue normally to next spiral point
        self.trajectory = Trajectory(np.array([]))
        context.node.get_logger().info("Running A*...")
        context.rover.send_drive_command(Twist())  # stop while planning
        try:
            occupancy_list = self.astar.a_star(rover_position_in_map, self.get_target_position(context=context))

        except SpiralEnd:
            # TODO: what to do in this case
            self.trajectory.reset()
            occupancy_list = None

        except NoPath:
            # increment end point
            if self.trajectory.increment_point():
                # TODO: what to do in this case
                self.trajectory.reset()
            occupancy_list = None

        if occupancy_list is None:
            self.trajectory = Trajectory(np.array([]))
        else:
            cartesian_coords = self.astar.ij_to_cartesian(np.array(occupancy_list))
            self.trajectory = Trajectory(
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
        self.UPDATE_DELAY = context.node.get_parameter("search.update_delay").value
        self.SAFE_APPROACH_DISTANCE = context.node.get_parameter("search.safe_approach_distance").value
        self.USE_COSTMAP = context.node.get_parameter("search.use_costmap").value
        self.STOP_THRESH = context.node.get_parameter("single_tag.stop_threshold").value
        self.DRIVE_FWD_THRESH = context.node.get_parameter("waypoint.drive_forward_threshold").value

        if not self.is_recovering:
            self.prev_target_pos_in_map = None

        origin_in_map = context.course.current_waypoint_pose_in_map().translation()[0:2]
        self.astar = AStar(origin_in_map, context)
        context.node.get_logger().info(f"Origin: {origin_in_map}")
        self.trajectory = Trajectory(coordinates=np.array([]))
        self.time_last_updated = context.node.get_clock().now()
        self.path_pub = context.node.create_publisher(Path, "path", 10)

    def on_exit(self, context: Context) -> None:
        pass

    def get_target_position(self, context: Context) -> np.ndarray | None:
        return context.env.current_target_pos()

    def determine_next(self, context: Context, is_finished: bool) -> State:
        if is_finished:
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

        context.node.get_logger().info("running approach")

        target_position = self.get_target_position(context)
        if target_position is None:
            from .long_range import LongRangeState

            if isinstance(self, LongRangeState) and not context.env.arrived_at_waypoint:
                return waypoint.WaypointState()

            return search.SearchState()

        rover_in_map = context.rover.get_pose_in_map()
        assert rover_in_map is not None

        if self.USE_COSTMAP:
            if context.node.get_clock().now() - self.time_last_updated > Duration(seconds=self.UPDATE_DELAY):
                context.node.get_logger().info(f"Generating new A-Star path")
                self.generate_astar_path(context)
                self.time_last_updated = context.node.get_clock().now()
            if len(self.trajectory.coordinates) - self.trajectory.cur_pt != 0: 
                target_position = self.trajectory.get_current_point()

        cmd_vel, has_arrived = context.drive.get_drive_command(
            target_position,
            rover_in_map,
            context.node.get_parameter("single_tag.stop_threshold").value,
            context.node.get_parameter("waypoint.drive_forward_threshold").value,
        )

        next_state = self.determine_next(context, has_arrived)
        if has_arrived:
            context.env.arrived_at_target = True
            context.env.last_target_location = self.get_target_position(context)
            context.course.increment_waypoint()
        else:
            context.rover.send_drive_command(cmd_vel)
        

        return next_state
