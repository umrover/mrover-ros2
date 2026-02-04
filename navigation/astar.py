import heapq
import math
from threading import Lock

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Header
from navigation.context import Context
from navigation.trajectory import Trajectory
from navigation.coordinate_utils import ij_to_cartesian, cartesian_to_ij, d_calc, segment_path, is_high_cost_point
from rclpy.publisher import Publisher


class SpiralEnd(Exception):
    """
    Raised when there are no more points left in the search spiral.
    """

    pass


class NoPath(Exception):
    """
    Raised when an A* path cannot be found.
    """

    pass


class OutOfBounds(Exception):
    """
    Raised when the rover or its destination is out of bounds of the costmap
    """

    pass


class DestinationInHighCost(Exception):
    """
    Raised when the destination is in high cost
    """

    pass


class AStar:
    context: Context
    costmap_lock: Lock

    path_pub: Publisher
    TRAVERSABLE_COST: float
    COSTMAP_THRESH: float
    ANGLE_THRESH: float
    USE_COSTMAP: bool
    USE_PURE_PURSUIT: bool

    def __init__(self, context: Context) -> None:
        self.context = context
        self.costmap_lock = Lock()

        self.path_pub = self.context.node.create_publisher(Path, "astar_path", 10)
        self.TRAVERSABLE_COST = self.context.node.get_parameter("search.traversable_cost").value
        self.COSTMAP_THRESH = self.context.node.get_parameter("costmap.costmap_thresh").value
        self.ANGLE_THRESH = self.context.node.get_parameter("search.angle_thresh").value

        if hasattr(context, "course") and context.course is not None:
            current_waypoint = context.course.current_waypoint()
            self.USE_PURE_PURSUIT = context.node.get_parameter_or("pure_pursuit.use_pure_pursuit", True).value
            if current_waypoint is None:
                self.USE_COSTMAP = context.node.get_parameter("costmap.use_costmap").value
            else:
                self.USE_COSTMAP = (
                    context.node.get_parameter("costmap.use_costmap").value or current_waypoint.enable_costmap
                )

    @staticmethod
    def return_path(self, came_from: dict[tuple, tuple], current_pos: tuple):
        """
        Reconstruct path after the A* search algorithm finishes.
        :param came_from: Dictionary mapping from each node to where we came from.
        :param current_pos: (i, j) coordinates of the current position.
        :return: List of (i, j) coordinates representing the shortest path from start to finish.
        """
        path = []
        pos = current_pos
        while pos:
            path.append(pos)
            if pos not in came_from:
                break
            pos = came_from[pos]
        return path[::-1]  # Reverse the path

    def chebyshev_calc(self, start: tuple, end: tuple) -> float:
        min_diag = min(
            abs(start[0] - end[0]),
            abs(start[1] - end[1]),
        ) * math.sqrt(2)
        straight_line = (
            max(
                abs(start[0] - end[0]),
                abs(start[1] - end[1]),
            )
            - min_diag
        )
        return min_diag + straight_line

    def a_star(
        self, start: np.ndarray, end: np.ndarray, context: Context | None = None, debug: bool = False
    ) -> list | None:
        """
        A* Algorithm: Find a path from start to end in the costmap using f(n) = g(n) + h(n).
        :param start: Rover pose in cartesian coordinates.
        :param end: Next point in the spiral trajectory in cartesian coordinates.
        :param debug: If True, returns the path at each step for debugging.
        :return: List of A* coordinates in occupancy grid (i, j) or None if start equals end.
        :raises NoPath: If no path to the goal can be found.
        """
        with self.costmap_lock:
            costmap2d = self.context.env.cost_map.data

            # Convert start and end to occupancy grid coordinates
            start_ij = tuple(cartesian_to_ij(self.context, start))
            end_ij = tuple(cartesian_to_ij(self.context, end))

            if start_ij == end_ij:
                return [end_ij]

            came_from: dict[tuple, tuple] = {}
            g_scores: dict[tuple, float] = {start_ij: 0.0}
            f_scores: dict[tuple, float] = {start_ij: d_calc(start_ij, end_ij)}

            open_set: list[tuple] = []
            heapq.heappush(open_set, (f_scores[start_ij], start_ij))

            adjacent_squares = np.array([[0, -1], [0, 1], [-1, 0], [1, 0], [-1, -1], [-1, 1], [1, -1], [1, 1]])

            debug_list = []

            while open_set:
                _, current = heapq.heappop(open_set)

            if debug:
                debug_list.append(self.return_path(self,came_from, current))

            if current == end_ij:
                return debug_list if debug else self.return_path(self,came_from, current)

                for rel_pos in adjacent_squares:
                    neighbor_pos = tuple(np.array(current) + rel_pos)

                    # Ensure within grid bounds
                    if not (0 <= neighbor_pos[0] < costmap2d.shape[0] and 0 <= neighbor_pos[1] < costmap2d.shape[1]):
                        continue

                    d = 1.0 if rel_pos[0] == 0 or rel_pos[1] == 0 else np.sqrt(2)
                    cost = max(costmap2d[neighbor_pos[0], neighbor_pos[1]], 1)

                    # TODO: Decide if we want to perform a mean filter here or in the callback. Probably best to do so in the callback

                    tentative_g_score = g_scores[current] + d * cost

                    if neighbor_pos not in g_scores or tentative_g_score < g_scores[neighbor_pos]:
                        came_from[neighbor_pos] = current
                        g_scores[neighbor_pos] = tentative_g_score
                        f_scores[neighbor_pos] = tentative_g_score + d_calc(neighbor_pos, end_ij)

                        # Only push to open_set if not already in it
                        if neighbor_pos not in (pos[1] for pos in open_set):
                            heapq.heappush(open_set, (f_scores[neighbor_pos], neighbor_pos))

            raise NoPath("No path could be found to the destination.")

    def use_astar(self, context: Context, dest: np.ndarray) -> bool:
        rover_in_map = context.rover.get_pose_in_map()

        # If no rover pose, no trajectory, or not enough points in star_traj, skip
        if rover_in_map is None or not self.USE_COSTMAP:
            return False

        straight_path = segment_path(context=context, dest=dest[:2], seg_len=context.env.cost_map.resolution)
        for point in straight_path.coordinates:
            if is_high_cost_point(point=point, context=context):
                return True
        return False

    def generate_trajectory(self, context: Context, dest: np.ndarray) -> Trajectory:
        """
        Generates a Trajectory object for the path from the rover's current position to the destination.
        :param context: Context object.
        :param dest: Destination point in cartesian coordinates.
        :return: Trajectory object.
        """
        start_time = context.node.get_clock().now()
        rover_SE3 = context.rover.get_pose_in_map()
        if rover_SE3 is None:
            context.node.get_logger().warn("Rover has no pose, cannot astar...")
            context.rover.send_drive_command(Twist())
            return Trajectory(np.array([]))

        rover_position_in_map = rover_SE3.translation()[:2]

        if not self.USE_COSTMAP or not self.use_astar(dest=dest):
            if (not self.USE_PURE_PURSUIT):
                return Trajectory(np.array([dest]))
            else:
                return Trajectory(np.array([rover_SE3.translation(), dest]))

        costmap_length = self.context.env.cost_map.data.shape[0]
        rover_ij = cartesian_to_ij(context, rover_position_in_map)
        dest_ij = cartesian_to_ij(context, dest)

        if not (0 <= int(dest_ij[0]) < costmap_length and 0 <= int(dest_ij[1]) < costmap_length) or not (
            0 <= int(rover_ij[0]) < costmap_length and 0 <= int(rover_ij[1]) < costmap_length
        ):
            raise OutOfBounds("Rover/destination outside of costmap")

        trajectory = Trajectory(np.array([]))
        context.rover.send_drive_command(Twist())  # Stop while planning

        try:
            occupancy_list = self.a_star(rover_position_in_map, dest)
        except SpiralEnd:
            trajectory.reset()
            occupancy_list = None
        except NoPath:
            context.node.get_logger().info("No path found")
            trajectory.reset()
            return Trajectory(np.array([]))
            # raise NoPath

        if occupancy_list is not None:
            cartesian_coords = ij_to_cartesian(context, np.array(occupancy_list))
            # Exclude the first point since it should be the rover's starting position
            if len(cartesian_coords) >= 1:
                trajectory = Trajectory(np.hstack((cartesian_coords, np.zeros((cartesian_coords.shape[0], 1)))))

            # Publish the path for visualization in RViz
            path_msg = Path()
            path_msg.header = Header()
            path_msg.header.frame_id = "map"

            poses = []
            for coord in cartesian_coords:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "map"
                pose_stamped.pose = Pose(
                    position=Point(x=coord[0], y=coord[1], z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                )
                poses.append(pose_stamped)

            path_msg.poses = poses

            if self.path_pub is not None:
                self.path_pub.publish(path_msg)
        else:
            # If occupancy_list is None, no path is needed (start == end).
            trajectory = Trajectory(np.array([]))

        context.node.get_logger().info(
            f"A-Star pathplanning took {(context.node.get_clock().now() - start_time).nanoseconds / 10e9} seconds"
        )
        return Trajectory(trajectory.coordinates[1:]) if len(trajectory.coordinates) > 1 else trajectory
