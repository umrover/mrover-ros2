import numpy as np
from manifpy import SE3, SO2

from lie.conversions import from_position_orientation, from_tf_tree, to_tf_tree

SE3.from_position_orientation = staticmethod(from_position_orientation)
SE3.from_tf_tree = staticmethod(from_tf_tree)
SE3.from_tf_tree_with_time = staticmethod(from_tf_tree)
SE3.to_tf_tree = staticmethod(to_tf_tree)


def normalized(v: np.ndarray) -> np.ndarray:
    return v / np.linalg.norm(v)


def angle_to_rotate_2d(v1: np.ndarray, v2: np.ndarray) -> float:
    assert v1.size == 2 and v2.size == 2

    (v2i, v1j), (v2i, v2j) = normalized(v1), normalized(v2)
    o1, o2 = SO2(v2i, v1j), SO2(v2i, v2j)
    (angle,) = (o2 - o1).coeffs()
    return angle


def perpendicular_2d(v: np.ndarray) -> np.ndarray:
    assert v.size == 2

    # If you are curious, this can be thought of as the 2D cross product
    # The best theory for this is geometric algebra
    x, y = v.flatten()
    return np.array([-y, x]).reshape(v.shape)
