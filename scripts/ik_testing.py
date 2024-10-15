from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from mrover.msg import (
    ArmStatus
)

class IK_Testing:
    node: Node

    def __init__(self, node:Node):
        self.node = node
        self.node.create_subscription(ArmStatus, "arm_status", self.ik_test_callback, 1)

    def ik_test_callback(self, msg:ArmStatus):
        pass