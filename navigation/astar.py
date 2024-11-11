import heapq
from threading import Lock

import numpy as np
from navigation.trajectory import Trajectory
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
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
    TRAVERSABLE_COST: float
    A_STAR_THRESH: float
    path_pub: Publisher

    def __init__(self, origin: np.ndarray, context: Context) -> None:
        self.origin = origin
        self.context = context
        self.path_pub = context.node.create_publisher(Path, "astar_path", 10)
        self.TRAVERSABLE_COST = self.context.node.get_parameter("search.traversable_cost").value
        self.A_STAR_THRESH = self.context.node.get_parameter("search.a_star_thresh").value
        self.costmap_lock = Lock()
    

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

    # distance heuristic using euclidean distance
    def d_calc(self, start: tuple, end: tuple) -> float:
        return np.sqrt((start[0] - end[0]) ** 2 + (start[1] - end[1]) ** 2)

    # TODO: do we want a filtered path that is every other position?
    def return_path(self, came_from: dict[tuple, tuple], current_pos: tuple):
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
        A-STAR Algorithm: f(n) = g(n) + h(n) to find a path from the given start to the given end in the given costmap
        :param start: rover pose in cartesian coordinates
        :param end: next point in the spiral from traj in cartesian coordinates
        :return: list of A-STAR coordinates in the occupancy grid coordinates (i,j)
        """
        with self.costmap_lock:
            costmap2d = self.context.env.cost_map.data
            # convert start and end to occupancy grid coordinates
            start_ij = tuple(self.cartesian_to_ij(start))
            end_ij = tuple(self.cartesian_to_ij(end))


            if start_ij == end_ij:
                return None
            
            came_from: dict[tuple, tuple] = {}

            g_scores: dict[tuple, float] = {}
            g_scores[start_ij] = 0.0

            f_scores: dict[tuple, float] = {}
            f_scores[start_ij] = self.d_calc(start_ij, end_ij)

            # priority queue of open nodes sorted based on f score
            open_set: list[tuple] = []
            heapq.heappush(open_set, (f_scores[start_ij], start_ij))
            
            # a node's neighbors relative positions
            adjacent_squares = np.array([[0, -1], [0, 1], [-1, 0], [1, 0], [-1, -1], [-1, 1], [1, -1], [1, 1]])

            debug_list: list = []
            while open_set:
                curr = heapq.heappop(open_set)[1]
                if debug: debug_list.append(self.return_path(came_from, curr))
                if curr == end_ij:
                    if debug: return debug_list
                    return self.return_path(came_from, curr)


                for rel_pos in adjacent_squares:
                    neighbor_pos = tuple(np.asarray(curr) + rel_pos)
                
                    # make sure within range
                    if (
                        neighbor_pos[0] > (costmap2d.shape[0] - 1)
                        or neighbor_pos[0] < 0
                        or neighbor_pos[1] > (costmap2d.shape[1] - 1)
                        or neighbor_pos[1] < 0
                    ):
                        continue

                    # TODO: make sure that costmap is in (x, y) format and not (row, col)
                    # make sure it is traversable terrain (not too high of a cost), skip if greater than or equal to traversable cost
                    if costmap2d[neighbor_pos[0], neighbor_pos[1]] >= self.TRAVERSABLE_COST:  # TODO: find optimal value
                        continue
                    
                    tentative_g_score = g_scores[curr] + self.d_calc(curr, neighbor_pos)

                    if neighbor_pos not in g_scores or tentative_g_score < g_scores[neighbor_pos]:
                        came_from[neighbor_pos] = curr
                        g_scores[neighbor_pos] = tentative_g_score
                        f_scores[neighbor_pos] = tentative_g_score + self.d_calc(neighbor_pos, end_ij)
                        if neighbor_pos not in (pos[1] for pos in open_set):
                            heapq.heappush(open_set, (f_scores[neighbor_pos], neighbor_pos))
            raise NoPath()
        
    def generate_trajectory(self, context: Context, dest: np.ndarray) -> Trajectory:
        rover_position_in_map = context.rover.get_pose_in_map().translation()[0:2]

        # If path to next spiral point has minimal cost per cell, continue normally to next spiral point
        trajectory = Trajectory(np.array([]))
        context.rover.send_drive_command(Twist())  # stop while planning
        try:
            occupancy_list = self.a_star(rover_position_in_map, dest)

        except SpiralEnd:
            # TODO: what to do in this case
            trajectory.reset()
            occupancy_list = None

        except NoPath:
            # increment end point
            if trajectory.increment_point():
                # TODO: what to do in this case
                trajectory.reset()
            occupancy_list = None

        if occupancy_list is None:
            trajectory = Trajectory(np.array([]))
        else:
            cartesian_coords = self.ij_to_cartesian(np.array(occupancy_list))
            trajectory = Trajectory(
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
        return trajectory

    def use_astar(self, context: Context, star_traj: Trajectory, trajectory: np.ndarray | None) -> bool:
        rover_in_map = context.rover.get_pose_in_map()
        follow_astar: bool
        if rover_in_map is None or \
            trajectory is None or \
            len(star_traj.coordinates) < 4: 
            follow_astar = False

        else: 
            astar_dist = 0.0
            for i in range(len(star_traj.coordinates[:-1])):
                astar_dist += self.d_calc(star_traj.coordinates[i], star_traj.coordinates[i+1])

            eucl_dist = self.d_calc(context.rover.get_pose_in_map().translation()[0:2], tuple(trajectory))

            follow_astar = abs(astar_dist - eucl_dist) / eucl_dist > self.A_STAR_THRESH
        return follow_astar
            
        
        
