import py_trees
from py_trees.common import Status
from py_trees.trees import BehaviourTree
from rclpy.node import Node
from navigation.context import Context
import py_trees_ros
from collections.abc import Callable
 
class BehaviorTree:
    node: Node
    name: str
    tree: BehaviourTree
    ctx: Context
 
    def __init__(
        self,
        node: Node,
        name: str,
        ctx: Context,
        root: py_trees.behaviour.Behaviour,
    ) -> None:
        self.node = node
        self.name = name
        self.ctx = ctx

        self.tree = BehaviourTree(root)
        self.tree.setup(timeout=15) # TODO: Make timeout a parameter

        self.node.get_logger().info("Behavior Tree Setup Finished")
 
    def tick(self) -> None:
        try:
            self.tree.tick()
        except Exception as e:
            self.logger.error(f"{self.name} behavior tree: Execption occured: {e}")
            self.shutdown()
 
    def shutdown(self) -> None:
        try:
            self.root.stop(Status.INVALID)
        except Exception as e:
            self.logger.error(f"{self.name} behavior tree: A leaf failed while the tree was being stopped: {e}")

class Leaf(py_trees.behaviour.Behaviour):
    node: Node
    ctx: Context

    def __init__(self, name: str) -> None:
        super().__init__(name=name)

    def setup(self, **kwargs) -> None:
        self.node = kwargs['node']
        self.ctx = kwargs['ctx']


class ConditionalNode(Leaf):
    cond: Callable[..., bool]
    val: bool


    def __init__(self, name: str, cond: Callable[..., bool], val: bool = True) -> None:
        super().__init__(name=name)
        self.cond = cond
        self.val = val

    def initialise(self) -> None:
        # Can't think of anything to reset for a conditional node
        return super().initialise()

    def update(self) -> Status:
        if (self.cond() == self.val):
            return Status.SUCCESS
        else:
            return Status.FAILURE



class ActionNode(Leaf):

    def __init__(self, node: Node, ctx: Context, name: str) -> None:
        super().__init__(node, ctx, name)


action = ActionNode();