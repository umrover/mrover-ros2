import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from rclpy.publisher import Publisher
from navigation.trajectory import Trajectory

def gen_marker(
    time: Time, point=[0.0, 0.0], color=[1.0, 1.0, 1.0], size=0.2, lifetime=5, id=0, delete=False
) -> Marker:
    """
    Creates and publishes a single spherical marker at the specified (x, y, z) coordinates.

    :param point: A tuple or list containing the (x, y) coordinates of the marker.
                The Z coordinate is set to 0.0 by default.
    :param time: The time object is provided by a node clock for setting the timestamp.

    :return: A Marker object representing the spherical marker with predefined size and color.
    """

    marker = Marker()
    marker.lifetime = Duration(seconds=lifetime).to_msg()
    marker.header = Header(frame_id="map")
    marker.header.stamp = time.to_msg()

    marker.ns = "single_point"
    marker.id = id
    marker.type = Marker.SPHERE

    if delete:
        marker.action = Marker.DELETEALL
        return marker

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

def ring_marker(
    time: Time, point=[0.0, 0.0], color=[1.0, 1.0, 1.0], radius=0.5, size=0.1, lifetime=5, id=0, delete=False
) -> Marker:
    """
    Creates and publishes a ring around the specified (x, y, z) coordinates with the given radius.

    :param point: A tuple or list containing the (x, y) coordinates of the marker.
                The Z coordinate is set to 0.0 by default.
    :param time: The time object is provided by a node clock for setting the timestamp.

    :return: A Marker object representing the spherical marker with predefined size and color.
    """

    marker = Marker()
    marker.lifetime = Duration(seconds=lifetime).to_msg()
    marker.header = Header(frame_id="map")
    marker.header.stamp = time.to_msg()

    marker.ns = "single_ring"
    marker.id = id
    marker.type = Marker.LINE_STRIP

    if delete:
        marker.action = Marker.DELETEALL
        return marker

    marker.action = Marker.ADD

    # Set the scale (size) of the ring
    marker.scale.x = size

    # Use line segments to create a ring
    num_points = 100
    for i in range(num_points):
        angle = 2 * np.pi * i / (num_points - 1)
        p = Point()
        p.x = point[0] + radius * np.cos(angle)
        p.y = point[1] + radius * np.sin(angle)
        p.z = point[2]
        marker.points.append(p)

    # Set the color (RGBA)
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0  # fully opaque

    return marker
