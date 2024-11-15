import heapq
from threading import Lock

import numpy as np
from navigation.trajectory import Trajectory
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from navigation import waypoint
from rclpy.publisher import Publisher
from nav_msgs.msg import Path
from std_msgs.msg import Header

from navigation.context import Context

class SpiralEnd(Exception):
    """
    Raise when there are no more points left in the search spiral
    """
    
    pass


class NoPath(Exception):
    """
    Raise when an A* path could not be found
    """

    pass

class AStar:
    origin: np.ndarray  # Holds the initial rover pose (waypoint of water bottle)
    context: Context
    costmap_lock: Lock

    path_pub: Publisher
    TRAVERSABLE_COST: float
    A_STAR_THRESH: float
    COSTMAP_THRESH: float

    def __init__(self, origin: np.ndarray, context: Context) -> None:
        self.origin = origin
        self.context = context
        self.costmap_lock = Lock()

        try:
            self.path_pub = context.node.create_publisher(Path, "astar_path", 10)
            self.TRAVERSABLE_COST = self.context.node.get_parameter("search.traversable_cost").value
            self.A_STAR_THRESH = self.context.node.get_parameter("search.a_star_thresh").value
            self.COSTMAP_THRESH = self.context.node.get_parameter("search.costmap_thresh").value
        except:
            pass
    

    def cartesian_to_ij(self, cart_coord: np.ndarray) -> np.ndarray:
        """
        Convert real world cartesian coordinates (x, y) to coordinates in the occupancy grid (i, j)
        using formula floor(v - (WP + [-W/2, H/2]) / r) * [1, -1]
        v: (x,y) coordinate
        WP: origin
        W, H: grid width, height (meters)
        r: resolution (meters/cell)
        :param cart_coord: array of x and y cartesian coordinates
        :return: array of i and j coordinates for the occupancy grid
        """
        return np.floor(
            (cart_coord[0:2] - self.context.env.cost_map.origin) / self.context.env.cost_map.resolution
        ).astype(np.int8)

    def ij_to_cartesian(self, ij_coords: np.ndarray) -> np.ndarray:
        """
        Convert coordinates in the occupancy grid (i, j) to real world cartesian coordinates (x, y)
        using formula (WP - [W/2, H/2]) + [j * r, i * r] + [r/2, -r/2] * [1, -1]
        WP: origin
        W, H: grid width, height (meters)
        r: resolution (meters/cell)
        :param ij_coords: array of i and j occupancy grid coordinates
        :return: array of x and y coordinates in the real world
        """
        half_res = np.array([self.context.env.cost_map.resolution / 2, self.context.env.cost_map.resolution / 2])
        return self.context.env.cost_map.origin + ij_coords * self.context.env.cost_map.resolution + half_res

    def d_calc(self, start: tuple, end: tuple) -> float:
        """
        Distance heuristic using euclidean distance.
        :param start: tuple of (i, j) coordinates for start node
        :param end: tuple of (i, j) coordinates for end node
        :return: euclidean distance between start and end nodes
        """
        return np.sqrt((start[0] - end[0]) ** 2 + (start[1] - end[1]) ** 2)

    def return_path(self, came_from: dict[tuple, tuple], current_pos: tuple):
        """
        Reconstructs path after the A* search algorithm has finished.
        :param came_from: dictionary of came_from nodes
        :param current_pos: tuple of (i, j) coordinates of the current position
        :return: list of (i, j) coordinates representing the shortest path from start to finish
        """
        path: list[tuple] = []
        pos = current_pos
        while pos:
            path.append(pos)
            if pos not in came_from: break
            pos = came_from[pos]
        reversed_path = path[::-1]

        filter_n = 1
        return reversed_path

    def a_star(self, start: np.ndarray, end: np.ndarray, debug=False) -> list | None:
        """
        A* Algorithm: Find a path from the start to the end in the costmap using f(n) = g(n) + h(n).
        :param start: Rover pose in cartesian coordinates.
        :param end: Next point in the spiral trajectory in cartesian coordinates.
        :param debug: If True, return the path at each step for debugging purposes.
        :return: List of A* coordinates in occupancy grid (i, j) or None if start equals end.
        """
        with self.costmap_lock:
            costmap2d = self.context.env.cost_map.data
            # Convert start and end to occupancy grid coordinates
            start_ij = tuple(self.cartesian_to_ij(start))
            end_ij = tuple(self.cartesian_to_ij(end))

            # If start and end are the same, return None
            if start_ij == end_ij:
                return None
            
            came_from: dict[tuple, tuple] = {}  # Track the path
            g_scores: dict[tuple, float] = {start_ij: 0.0}  # Cost from start to the current node
            f_scores: dict[tuple, float] = {start_ij: self.d_calc(start_ij, end_ij)}  # Estimated cost from start to end

            # Priority queue of open nodes sorted based on f score
            open_set: list[tuple] = []
            heapq.heappush(open_set, (f_scores[start_ij], start_ij))
            
            # Relative positions of a node's neighbors
            adjacent_squares = np.array([[0, -1], [0, 1], [-1, 0], [1, 0], [-1, -1], [-1, 1], [1, -1], [1, 1]])

            debug_list: list = []  # List to store paths for debugging
            while open_set:
                curr = heapq.heappop(open_set)[1]
                if debug:
                    debug_list.append(self.return_path(came_from, curr))
                if curr == end_ij:
                    return debug_list if debug else self.return_path(came_from, curr)

                for rel_pos in adjacent_squares:
                    neighbor_pos = tuple(np.asarray(curr) + rel_pos)
                
                    # Ensure within grid bounds
                    if (
                        neighbor_pos[0] >= costmap2d.shape[0] - 1
                        or neighbor_pos[0] < 1
                        or neighbor_pos[1] >= costmap2d.shape[1] - 1
                        or neighbor_pos[1] < 1
                    ):
                        continue

                    # Check if terrain is traversable
                    if costmap2d[neighbor_pos[0], neighbor_pos[1]] >= self.TRAVERSABLE_COST:
                        continue
                    
                    # TODO: Fix this, it works in the testing sim but doesn't work in the actual rover sim
                    # Check if dilated rover terrain is traversable
                    # edge_cost = False
                    # for rel_edge in adjacent_squares:
                    #     edge = neighbor_pos + rel_edge
                    #     if costmap2d[edge[0], edge[1]] >= self.TRAVERSABLE_COST:
                    #         edge_cost = True
                    #         break
                    # if edge_cost:
                    #     continue

                    tentative_g_score = g_scores[curr] + self.d_calc(curr, neighbor_pos)

                    if neighbor_pos not in g_scores or tentative_g_score < g_scores[neighbor_pos]:
                        came_from[neighbor_pos] = curr
                        g_scores[neighbor_pos] = tentative_g_score
                        f_scores[neighbor_pos] = tentative_g_score + self.d_calc(neighbor_pos, end_ij)
                        if neighbor_pos not in (pos[1] for pos in open_set):
                            heapq.heappush(open_set, (f_scores[neighbor_pos], neighbor_pos))
            raise NoPath()  # Raise exception if no path is found
        
    def generate_trajectory(self, context: Context, dest: np.ndarray) -> Trajectory:
        """
        Generates a trajectory object for the path from the rover's current position to the destination
        :param context: Context object
        :param dest: destination point
        :return: Trajectory object
        """
        # Get rover position in map
        rover_position_in_map = context.rover.get_pose_in_map().translation()[0:2]

        # Move the costmap if we are outside of the threshold for it
        costmap_length = self.context.env.cost_map.data.shape[0]
        thresh = (costmap_length * self.COSTMAP_THRESH, costmap_length * (1 - self.COSTMAP_THRESH))
        rover_ij = self.cartesian_to_ij(rover_position_in_map)

        if rover_ij[0] < thresh[0] or rover_ij[0] > thresh[1] \
            or rover_ij[1] < thresh[0] or rover_ij[1] > thresh[1]:
            context.move_costmap()

        # If path to next spiral point has minimal cost per cell, continue normally to next spiral point
        trajectory = Trajectory(np.array([]))
        context.rover.send_drive_command(Twist())  # stop while planning
        try:
            # Call the A* algorithm to generate a path
            occupancy_list = self.a_star(rover_position_in_map, dest)

        except SpiralEnd:
            # TODO: what to do in this case
            trajectory.reset()
            occupancy_list = None

        except NoPath:
            # increment end point
            context.node.get_logger().info("No path found")
            trajectory.reset()
            return trajectory

        if occupancy_list is None:
            # If no path was found, reset the trajectory
            trajectory = Trajectory(np.array([]))
        else:
            # Convert the occupancy list to cartesian coordinates
            cartesian_coords = self.ij_to_cartesian(np.array(occupancy_list))
            trajectory = Trajectory(
                np.hstack((cartesian_coords, np.zeros((cartesian_coords.shape[0], 1))))
            )  # Current point gets set back to 0

            # Create path type to publish planned path segments to see in rviz
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
        return trajectory

    def use_astar(self, context: Context, star_traj: Trajectory, trajectory: np.ndarray | None) -> bool:
        """
        Determine whether to follow the A* path based on the difference between the A* path distance
        and the Euclidean distance to the target.

        :param context: Context object containing rover and node information
        :param star_traj: A* trajectory to be evaluated
        :param trajectory: Current point in the trajectory
        :return: True if the A* path should be followed, False otherwise
        """
        rover_in_map = context.rover.get_pose_in_map()
        follow_astar: bool

        # If any required information is missing or A* trajectory has fewer than 4 coordinates, do not follow A* path
        if rover_in_map is None or trajectory is None or len(star_traj.coordinates) < 4:
            follow_astar = False
        else:
            # Calculate the total distance of the A* path
            astar_dist = 0.0
            for i in range(len(star_traj.coordinates[:-1])):
                astar_dist += self.d_calc(star_traj.coordinates[i], star_traj.coordinates[i + 1])

            # Calculate the Euclidean distance from the rover's position to the target
            eucl_dist = self.d_calc(context.rover.get_pose_in_map().translation()[0:2], tuple(trajectory))

            # Determine whether the relative difference between A* and Euclidean distances exceeds the threshold
            follow_astar = abs(astar_dist - eucl_dist) / eucl_dist > self.A_STAR_THRESH
        
        return follow_astar
            
        
        
