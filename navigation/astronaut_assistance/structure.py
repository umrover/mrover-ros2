# Build the tree and subtrees here

import py_trees
from py_trees import decorators
from py_trees.composites import Sequence, Selector
from navigation.astronaut_assistance import stay_leaves
from navigation.behavior_tree.behavior_tree import Leaf
from navigation.astronaut_assistance.astronaut_assistance import Command



def build_stay_subtree() -> py_trees.behaviour.Behaviour:
    """
    Returns the root of the stay command subtree
    """

    checkCmd: Leaf = stay_leaves.CheckCommand("Stay Command?", Command.STAY)
    stop = decorators.SuccessIsRunning("Success is Running", stay_leaves.Stop("Stop"))
    stay_root: Sequence = Sequence("stay", False, [checkCmd, stop])
    return stay_root
