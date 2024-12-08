import numpy as np
from navigation.context import Context
def cartesian_to_ij(context: Context, cart_coord: np.ndarray) -> np.ndarray:
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
            (cart_coord[0:2] - context.env.cost_map.origin) / context.env.cost_map.resolution
        ).astype(np.int8)

def ij_to_cartesian(context: Context, ij_coords: np.ndarray) -> np.ndarray:
    """
    Convert coordinates in the occupancy grid (i, j) to real world cartesian coordinates (x, y)
    using formula (WP - [W/2, H/2]) + [j * r, i * r] + [r/2, -r/2] * [1, -1]
    WP: origin
    W, H: grid width, height (meters)
    r: resolution (meters/cell)
    :param ij_coords: array of i and j occupancy grid coordinates
    :return: array of x and y coordinates in the real world
    """
    half_res = np.array([context.env.cost_map.resolution / 2, context.env.cost_map.resolution / 2])
    return context.env.cost_map.origin + ij_coords * context.env.cost_map.resolution + half_res

def d_calc(start: tuple, end: tuple) -> float:
    """
    Distance heuristic using euclidean distance.
    :param start: tuple of (i, j) coordinates for start node
    :param end: tuple of (i, j) coordinates for end node
    :return: euclidean distance between start and end nodes
    """
    return np.sqrt((start[0] - end[0]) ** 2 + (start[1] - end[1]) ** 2)

def vec_angle(self, v1: tuple, v2:tuple) -> float:
        """
        Calculates angle between two vectors
        """
        # Compute dot product and magnitudes
        dot_product = np.dot(v1, v2)
        magnitude_v1 = np.linalg.norm(v1)
        magnitude_v2 = np.linalg.norm(v2)
    
        # Calculate cosine of the angle
        cos_theta = dot_product / (magnitude_v1 * magnitude_v2)
        
        # Clamp the value to avoid numerical errors outside the range [-1, 1]
        cos_theta = np.clip(cos_theta, -1.0, 1.0)

        #Compute the angle in radians
        angle_rad = np.arccos(cos_theta)

        return abs(angle_rad)