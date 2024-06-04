from manifpy import SE3

from .conversions import from_position_orientation, from_tf_tree, to_tf_tree

SE3.from_position_orientation = classmethod(from_position_orientation)
SE3.from_tf_tree = classmethod(from_tf_tree)
SE3.to_tf_tree = classmethod(to_tf_tree)

