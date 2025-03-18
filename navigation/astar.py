import heapq
import math
from threading import Lock

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Header
from navigation.context import Context
from navigation.trajectory import Trajectory
from navigation.coordinate_utils import ij_to_cartesian, cartesian_to_ij, d_calc
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


class AStar:
    context: Context
    costmap_lock: Lock

    path_pub: Publisher
    TRAVERSABLE_COST: float
    A_STAR_THRESH: float
    COSTMAP_THRESH: float
    ANGLE_THRESH: float

    def __init__(self, context: Context) -> None:
        self.context = context
        self.costmap_lock = Lock()

        # Attempt to retrieve parameters. If they don't exist, set them to None or skip.
        try:
            self.path_pub = self.context.node.create_publisher(Path, "astar_path", 10)
            self.TRAVERSABLE_COST = self.context.node.get_parameter("search.traversable_cost").value
            self.A_STAR_THRESH = self.context.node.get_parameter("search.a_star_thresh").value
            self.COSTMAP_THRESH = self.context.node.get_parameter("search.costmap_thresh").value
            self.ANGLE_THRESH = self.context.node.get_parameter("search.angle_thresh").value
        except Exception as e:
            self.path_pub = None
            self.TRAVERSABLE_COST = 0.0
            self.A_STAR_THRESH = 0.0
            self.COSTMAP_THRESH = 0.0
            self.ANGLE_THRESH = 0.0
            self.context.node.get_logger().info(f"Parameter retrieval failed: {e}. Setting to default values")

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

    def vec_angle(self, v1: tuple, v2: tuple) -> float:
        """
        Calculate the angle between two vectors (v1, v2).
        :param v1: Tuple (x1, y1).
        :param v2: Tuple (x2, y2).
        :return: Absolute angle (in radians) between the two vectors.
        """
        dot_product = np.dot(v1, v2)
        magnitude_v1 = np.linalg.norm(v1)
        magnitude_v2 = np.linalg.norm(v2)

        if magnitude_v1 == 0 or magnitude_v2 == 0:
            return 0.0

        cos_theta = dot_product / (magnitude_v1 * magnitude_v2)
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        angle_rad = np.arccos(cos_theta)

        return abs(angle_rad)

    def a_star(self, start: np.ndarray, end: np.ndarray, debug: bool = False) -> list | None:
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
                    debug_list.append(self.return_path(came_from, current))

                if current == end_ij:
                    return debug_list if debug else self.return_path(came_from, current)

                for rel_pos in adjacent_squares:
                    neighbor_pos = tuple(np.array(current) + rel_pos)

                    # Ensure within grid bounds
                    if not (
                        1 <= neighbor_pos[0] < costmap2d.shape[0] - 1 and 1 <= neighbor_pos[1] < costmap2d.shape[1] - 1
                    ):
                        continue

                    # Check if terrain is traversable
                    if costmap2d[neighbor_pos[0], neighbor_pos[1]] >= self.TRAVERSABLE_COST:
                        continue

                    tentative_g_score = g_scores[current] + d_calc(current, neighbor_pos)

                    if neighbor_pos not in g_scores or tentative_g_score < g_scores[neighbor_pos]:
                        came_from[neighbor_pos] = current
                        g_scores[neighbor_pos] = tentative_g_score
                        f_scores[neighbor_pos] = tentative_g_score + d_calc(neighbor_pos, end_ij)

                        # Only push to open_set if not already in it
                        if neighbor_pos not in (pos[1] for pos in open_set):
                            heapq.heappush(open_set, (f_scores[neighbor_pos], neighbor_pos))

            raise NoPath("No path could be found to the destination.")

    def use_astar(self, context: Context, star_traj: Trajectory, dest: np.ndarray | None) -> bool:
        """
        Decide whether to follow the A* path based on the difference between the A* path distance
        and a lower-bound distance (Chebyshev-based or Euclidean).
        :param context: Context object containing rover and node information.
        :param star_traj: A* trajectory to be evaluated.
        :param trajectory: Current point in the trajectory (destination).
        :return: True if the A* path should be followed, False otherwise.
        """
        rover_in_map = context.rover.get_pose_in_map()
        follow_astar = True

        # If no rover pose, no trajectory, or not enough points in star_traj, skip
        if (
            rover_in_map is None
            or len(star_traj.coordinates) < 1
            or not self.context.node.get_parameter("search.use_costmap").value
        ):
            return False

        # Calculate actual A* distance
        astar_dist = 0.0
        for i in range(len(star_traj.coordinates) - 1):
            astar_dist += d_calc(star_traj.coordinates[i], star_traj.coordinates[i + 1])

        # Calculate a minimal possible path (a diagonal-first approach).
        min_diag = min(
            abs(star_traj.coordinates[0][0] - star_traj.coordinates[-1][0]),
            abs(star_traj.coordinates[0][1] - star_traj.coordinates[-1][1]),
        ) * math.sqrt(2)
        straight_line = max(
            abs(star_traj.coordinates[0][0] - star_traj.coordinates[-1][0]),
            abs(star_traj.coordinates[0][1] - star_traj.coordinates[-1][1]),
        ) - min(
            abs(star_traj.coordinates[0][0] - star_traj.coordinates[-1][0]),
            abs(star_traj.coordinates[0][1] - star_traj.coordinates[-1][1]),
        )
        min_astar = min_diag + straight_line

        # context.node.get_logger().info(f"minastar: {min_astar} act_astar: {astar_dist}")

        # Compare relative difference
        if min_astar > 0:
            follow_astar = abs(astar_dist - min_astar) / min_astar > self.A_STAR_THRESH
        else:
            # If min_astar is 0, it might mean start/end are the same
            follow_astar = True

        # context.node.get_logger().info("Following A* path" if follow_astar else "Not following A* path")
        return follow_astar

    def generate_trajectory(self, context: Context, dest: np.ndarray) -> Trajectory:
        """
        Generates a Trajectory object for the path from the rover's current position to the destination.
        :param context: Context object.
        :param dest: Destination point in cartesian coordinates.
        :return: Trajectory object.
        """
        rover_SE3 = context.rover.get_pose_in_map()
        assert rover_SE3 is not None
        rover_position_in_map = rover_SE3.translation()[:2]

        if not context.node.get_parameter("search.use_costmap").value:
            return Trajectory(np.array([dest]))

        costmap_length = self.context.env.cost_map.data.shape[0]
        threshold = (costmap_length * self.COSTMAP_THRESH, costmap_length * (1 - self.COSTMAP_THRESH))
        rover_ij = cartesian_to_ij(context, rover_position_in_map)

        # Move the costmap if we are outside of the threshold
        if not (threshold[0] <= rover_ij[0] <= threshold[1] and threshold[0] <= rover_ij[1] <= threshold[1]):
            if not context.node.get_parameter("custom_costmap").value:
                context.move_costmap()

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

        if occupancy_list is not None:
            cartesian_coords = ij_to_cartesian(context, np.array(occupancy_list))
            # Z=0 for all points in the path
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

        if self.use_astar(context=context, star_traj=trajectory, dest=dest):
            return trajectory
        else:
            return Trajectory(np.array([dest]))
