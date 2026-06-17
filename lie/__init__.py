import numpy as np
from manifpy import SE3, SE3Tangent, SO3, SO3Tangent, SE2, SE2Tangent, SO2, SO2Tangent

from lie.conversions import from_position_orientation, from_tf_tree, to_tf_tree, from_tf_tree_with_time

SE3.from_position_orientation = staticmethod(from_position_orientation)
SE3.from_tf_tree = staticmethod(from_tf_tree)
SE3.from_tf_tree_with_time = staticmethod(from_tf_tree_with_time)
SE3.to_tf_tree = staticmethod(to_tf_tree)


def normalized(v: np.ndarray) -> np.ndarray:
    return v / np.linalg.norm(v)


# For more mathy approaches to the following check out geometric algebra


def angle_to_rotate_2d(v1, v2):
    """
    :return: The angle in radians to rotate v1 to align with v2
    """
    assert v1.size == 2 and v2.size == 2

    # Determine the magnitude of the smallest angle between the two vectors
    # Pre-normalized the vectors, so their dot product is within the domain of inverse cosine
    smallest_angle = np.arccos(np.dot(normalized(v1), normalized(v2)))
    # Determine the sign of our rotation by checking the alignment of the perpendicular vector
    perp_alignment = np.dot(v2, perpendicular_2d(v1))
    return np.copysign(smallest_angle, perp_alignment)


def perpendicular_2d(v: np.ndarray) -> np.ndarray:
    assert v.size == 2

    x, y = v.flatten()
    return np.array([-y, x]).reshape(v.shape)  # "2D cross product"
