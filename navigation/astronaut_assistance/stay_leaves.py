import typing
import py_trees
from py_trees.common import Status, Access
from py_trees.trees import BehaviourTree
from py_trees.blackboard import Client
from rclpy.node import Node
from navigation.context import Context
from navigation.behavior_tree.behavior_tree import Leaf
from navigation.astronaut_assistance.astronaut_assistance import Command
from geometry_msgs.msg import Twist


class CheckCommand(Leaf):

    cmd: Command
    bb: Client

    def __init__(self, name: str, cmd: Command) -> None:
        super().__init__(name=name)
        self.cmd = cmd

    def setup(self, **kwargs: typing.Any) -> None:
        self.bb = self.attach_blackboard_client()
        self.bb.register_key(key="current_command", access=Access.READ)

    def update(self) -> Status:
        return Status.SUCCESS if self.bb.current_command == self.cmd else Status.FAILURE


class Stop(Leaf):

    def __init__(self, name: str) -> None:
        super().__init__(name=name)

    def update(self) -> Status:
        self.ctx.rover.send_drive_command(Twist())
        return Status.SUCCESS
