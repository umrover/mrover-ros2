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
            occupancy_list = self.astar.a_star(rover_position_in_map, context.course.current_waypoint_pose_in_map().translation())

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
        origin_in_map = context.course.current_waypoint_pose_in_map().translation()[0:2]
        self.astar = AStar(origin_in_map, context)
        self.time_last_updated = context.node.get_clock().now()
        self.USE_COSTMAP = context.node.get_parameter("search.use_costmap").value
        self.UPDATE_DELAY = context.node.get_parameter("search.update_delay").value
        self.trajectory = Trajectory(np.array([]))
        self.path_pub = context.node.create_publisher(Path, "path", 10)
        
        assert context.course is not None

        current_waypoint = context.course.current_waypoint()
        assert current_waypoint is not None

        context.env.arrived_at_waypoint = False

        # TODO(neven): add service to move costmap if going to watter bottle search
        # if current_waypoint.type.val == WaypointType.WATER_BOTTLE:
        #     context.node.get_logger().info("Moving cost map")
        #     client = context.node.create_client(MoveCostMap, "move_cost_map")
        #     while not client.wait_for_service(timeout_sec=1.0):
        #         context.node.get_logger().info("waiting for move_cost_map service...")
        #     req = MoveCostMap.Request()

        #     req.course = f"course{context.course.waypoint_index}"
        #     future = client.call_async(req)
        #     # TODO(neven): make this actually wait for the service to finish
        #     #context.node.get_logger().info("called thing")
        #     # rclpy.spin_until_future_complete(context.node, future)
        #     # while not future.done():
        #     #     pass
        #     # if not future.result():
        #         # context.node.get_logger().info("move_cost_map service call failed")
        #     context.node.get_logger().info("Moved cost map")
            

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

        # Attempt to find the waypoint in the TF tree and drive to it
        if not self.USE_COSTMAP:
            waypoint_position_in_map = context.course.current_waypoint_pose_in_map().translation()
            cmd_vel, arrived = context.drive.get_drive_command(
                    waypoint_position_in_map,
                    rover_in_map,
                    context.node.get_parameter("waypoint.stop_threshold").value,
                    context.node.get_parameter("waypoint.drive_forward_threshold").value,
                )

            if arrived:
                    if self.trajectory.increment_point():
                        context.env.arrived_at_waypoint = True
                        if current_waypoint.type.val != WaypointType.NO_SEARCH:
                            
                            if context.node.get_parameter("search.use_costmap").value:
                                # We finished a waypoint associated with the water bottle, but we have not seen it yet and are using the costmap to search
                                costmap_search_state = costmap_search.CostmapSearchState()
                                # water_bottle_search_state.new_trajectory(context)
                                return costmap_search_state

                            else:
                                search_state = search.SearchState()
                                return search_state
                            
                        else:
                            # We finished a regular waypoint, go onto the next one
                            context.course.increment_waypoint()

            if context.rover.stuck:
                context.rover.previous_state = self
                return recovery.RecoveryState()

            context.rover.send_drive_command(cmd_vel)



        else:
            context.node.get_logger().info("Using costmap to traverse to waypoint")
            if not hasattr(context.env.cost_map, 'data'): return self
            
            if context.node.get_clock().now() - self.time_last_updated > Duration(seconds=self.UPDATE_DELAY):
                context.node.get_logger().info(f"Generating new A-Star path")
                self.generate_astar_path(context)
                self.time_last_updated = context.node.get_clock().now()

            if len(self.trajectory.coordinates) - self.trajectory.cur_pt != 0: 
                waypoint_position_in_map = self.trajectory.get_current_point()
                cmd_vel, arrived = context.drive.get_drive_command(
                    waypoint_position_in_map,
                    rover_in_map,
                    context.node.get_parameter("waypoint.stop_threshold").value,
                    context.node.get_parameter("waypoint.drive_forward_threshold").value,
                )

                if arrived:
                    if self.trajectory.increment_point():
                        context.env.arrived_at_waypoint = True
                        if context.node.get_parameter("search.use_costmap").value and not current_waypoint.type.val == WaypointType.NO_SEARCH:
                            # We finished a waypoint associated with the water bottle, but we have not seen it yet and are using the costmap to search
                            costmap_search_state = costmap_search.CostmapSearchState()
                            # water_bottle_search_state.new_trajectory(context)
                            return costmap_search_state
                        else:
                            # We finished a regular waypoint, go onto the next one
                            context.course.increment_waypoint()

                if context.rover.stuck:
                    context.rover.previous_state = self
                    return recovery.RecoveryState()

                context.rover.send_drive_command(cmd_vel)

        return self
