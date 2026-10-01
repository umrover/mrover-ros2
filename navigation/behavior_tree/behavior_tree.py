import py_trees
from py_trees.common import Status
from py_trees.trees import BehaviourTree
from rclpy.node import Node
from navigation.context import Context
 
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

    def __init__(
        self,
        node: Node,
        ctx: Context,
        name: str,
    ) -> None:
        super().__init__(name=name)
        self.node = node
        self.ctx = ctx