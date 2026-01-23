import numpy as np

import rclpy
import transforms3d
from geometry_msgs.msg import Point, TransformStamped, Vector3
from sensor_msgs.msg import NavSatFix 

EARTH_RADIUS = 6371000
TRANSLATION_IDENTITY = [0.0, 0.0, 0.0]
ROTATION_IDENTITY = np.identity(3, dtype=np.float64)
ZOOM_IDENTITY = [1.0, 1.0, 1.0]
SHEAR_IDENTITY = TRANSLATION_IDENTITY


def gps_to_world(gps_coord: NavSatFix, ref_coord: NavSatFix, name: str, parent: str = "world") -> TransformStamped:
    """
    Returns the GPS to cartesian world transform.

    :param gps_coord: The gps coordinate that we want in the cartesian world frame
    :param name: The name of the returned transform frame
    :param parent: The name of the reference world frame
    """
    t = TransformStamped()
    t.header.stamp = rclpy.time.Time()
    t.header.frame_id = parent
    t.child_frame_id = name
    longitude_delta = gps_coord.longitude - ref_coord.longitude
    latitude_delta = gps_coord.latitude - ref_coord.latitude
    t.transform.translation.x = np.radians(longitude_delta) * EARTH_RADIUS
    t.transform.translation.y = np.radians(latitude_delta) * EARTH_RADIUS
    t.transform.translation.z = gps_coord.altitude
    return t


def vector3_to_point(vec3: Vector3) -> Point:
    return Point(x=vec3.x, y=vec3.y, z=vec3.z)


def point_to_vector3(pt: Point) -> Vector3:
    return Vector3(x=pt.x, y=pt.y, z=pt.z)

# taken from tf_transformations package src

def _reorder_output_quaternion(quaternion):
    """Reorder quaternion to have w term last."""
    w, x, y, z = quaternion
    return x, y, z, w

def quaternion_conjugate(quaternion):
    """
    Return conjugate of quaternion.

    >>> q0 = random_quaternion()
    >>> q1 = quaternion_conjugate(q0)
    >>> q1[3] == q0[3] and all(q1[:3] == -q0[:3])
    True

    """
    return np.array((-quaternion[0], -quaternion[1],
                        -quaternion[2], quaternion[3]), dtype=np.float64)


def quaternion_inverse(quaternion):
    """
    Return inverse of quaternion.

    >>> q0 = random_quaternion()
    >>> q1 = quaternion_inverse(q0)
    >>> numpy.allclose(quaternion_multiply(q0, q1), [0, 0, 0, 1])
    True

    """
    return quaternion_conjugate(quaternion) / np.dot(quaternion, quaternion)

def _reorder_input_quaternion(quaternion):
    """Reorder quaternion to have w term first."""
    x, y, z, w = quaternion
    return w, x, y, z

def quaternion_matrix(quaternion):
    """
    Return 4x4 homogeneous rotation matrix from quaternion.

    >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
    >>> numpy.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
    True

    """
    rotation_matrix = transforms3d.quaternions.quat2mat(
        _reorder_input_quaternion(quaternion)
    )
    return transforms3d.affines.compose(TRANSLATION_IDENTITY,
                                        rotation_matrix,
                                        ZOOM_IDENTITY)

def quaternion_from_matrix(matrix):
    """
    Return quaternion from rotation matrix.

    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095])
    True

    """
    rotation_matrix = transforms3d.affines.decompose(matrix)[1]
    return _reorder_output_quaternion(
        transforms3d.quaternions.mat2quat(rotation_matrix)
    )

def quaternion_multiply(quaternion1, quaternion0):
    """
    Return multiplication of two quaternions.

    >>> q = quaternion_multiply([1, -2, 3, 4], [-5, 6, 7, 8])
    >>> numpy.allclose(q, [-44, -14, 48, 28])
    True

    """
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    w2, x2, y2, z2 = transforms3d.quaternions.qmult([w1, x1, y1, z1],
                                                    [w0, x0, y0, z0])
    return x2, y2, z2, w2