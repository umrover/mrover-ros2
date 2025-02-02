import numpy as np
from navigation.context import Context
from visualization_msgs.msg import Marker
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Header
from navigation.trajectory import Trajectory
from lie import SE3


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
    return np.floor((cart_coord[0:2] - context.env.cost_map.origin) / context.env.cost_map.resolution).astype(np.int8)


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


def vec_angle(self, v1: tuple, v2: tuple) -> float:
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

    # Compute the angle in radians
    angle_rad = np.arccos(cos_theta)

    return abs(angle_rad)


def gen_marker(context: Context, point=[0.0, 0.0], color=[1.0, 1.0, 1.0], size=0.2, lifetime=5, id=0) -> Marker:
    """
    Creates and publishes a single spherical marker at the specified (x, y, z) coordinates.

    :param point: A tuple or list containing the (x, y) coordinates of the marker.
                The Z coordinate is set to 0.0 by default.
    :param context: The context object providing necessary ROS utilities,
                    such as the node clock for setting the timestamp.
    :return: A Marker object representing the spherical marker with predefined size and color.
    """

    marker = Marker()
    marker.lifetime = Duration(seconds=lifetime).to_msg()
    marker.header = Header(frame_id="map")
    marker.header.stamp = context.node.get_clock().now().to_msg()

    marker.ns = "single_point"
    marker.id = id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    # Set the scale (size) of the sphere
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size

    # Set the color (RGBA)
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0  # fully opaque

    # Define the position
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.position.z = 0.0

    # Orientation is irrelevant for a sphere but must be valid
    marker.pose.orientation.w = 1.0

    return marker


def segment_path(context: Context, dest: np.ndarray, seg_len: float = 1):
    """
    Segment the path from the rover's current position to the current waypoint into equally spaced points

    Args:
        context (Context): The global context object
        seg_len (float, optional): The length of each segment of the path. Defaults to 2.

    Returns:
        Trajectory: The segmented path
    """

    rover_SE3 = context.rover.get_pose_in_map()
    assert rover_SE3 is not None
    rover_translation = rover_SE3.translation()[0:2]

    # Create a numpy array with the rover's current position and the waypoint position
    traj_path = np.array([rover_translation, dest])

    # Calculate the number of segments needed for the path
    num_segments: int = int(np.ceil(d_calc(tuple(dest), rover_translation) // seg_len))

    # If there is more than one segment, create the segments
    if num_segments > 0:

        # Calculate the direction vector from the rover to the waypoint
        direction = (dest - rover_translation) / num_segments

        # Create the segments by adding the direction vector to the rover's position
        temp = np.array([rover_translation + i * direction for i in range(0, num_segments)])
        traj_path = np.concatenate((temp, np.array([dest])), axis=0)
        context.node.get_logger().info(f"Destination: {dest}")
        np.vstack((traj_path, dest))

    # Create a Trajectory object from the segmented path
    segmented_trajectory = Trajectory(np.hstack((traj_path, np.zeros((traj_path.shape[0], 1)))))

    context.node.get_logger().info(f"Segmented path: {segmented_trajectory.coordinates}")
    return segmented_trajectory


def is_high_cost_point(point: np.ndarray, context: Context, min_cost=0.2) -> bool:
    cost_map = context.env.cost_map.data

    point_ij = cartesian_to_ij(context=context, cart_coord=point)

    if not (0 <= int(point_ij[0]) < cost_map.shape[0] and 0 <= int(point_ij[1]) < cost_map.shape[1]):
        context.node.get_logger().warn("Point is out of bounds in the costmap")
        return False
    return cost_map[int(point_ij[0])][int(point_ij[1])] > min_cost
