# "main" file for astronaut assistance mission where the tree object is initialized and set up

import py_trees_ros
from navigation.behavior_tree import behavior_tree
import py_trees
from py_trees.common import Status
from py_trees.trees import BehaviourTree
from rclpy.node import Node
from navigation.context import Context
from enum import Enum

class Command(Enum):
    FOLLOW = 1,
    STAY = 2,
    FETCH = 3,
    COME = 4,
    GIVE = 5



