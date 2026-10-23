import py_trees
from py_trees.common import Status
from py_trees.trees import BehaviourTree
from rclpy.node import Node
from navigation.context import Context
import py_trees_ros
 
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



# ros trees tutorial stuff
def create_root():
    # behaviours
    root = py_trees.composites.Parallel("Tutorial")
    topics2bb = py_trees.composites.Sequence("Topics2BB")
    battery2bb = py_trees_ros.battery.ToBlackboard(name="Battery2BB",
                                                   topic_name="/battery/state",
                                                   threshold=30.0
                                                   )
    priorities = py_trees.composites.Selector("Priorities")
    battery_check = py_trees.meta.success_is_failure(py_trees.composites.Selector)(name="Battery Emergency")
    is_battery_ok = py_trees.blackboard.CheckBlackboardVariable(
        name="Battery Ok?",
        variable_name='battery_low_warning',
        expected_value=False
    )
    flash_led_strip = py_trees_ros.tutorials.behaviours.FlashLedStrip(
        name="FlashLEDs",
        colour="red")
    idle = py_trees.behaviours.Running(name="Idle")

    # tree
    root.add_children([topics2bb, priorities])
    topics2bb.add_child(battery2bb)
    priorities.add_children([battery_check, idle])
    battery_check.add_children([is_battery_ok, flash_led_strip])
    return root