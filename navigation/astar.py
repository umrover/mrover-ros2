import heapq
import random
from threading import Lock

import numpy as np

import rclpy

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

    def __init__(self, origin: np.ndarray, context: Context) -> None:
        self.origin = origin
        self.context = context

        self.TRAVERSABLE_COST = self.context.node.get_parameter("search.traversable_cost").value

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
        return path[::-1]

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
        
        
