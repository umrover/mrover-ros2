# ros2_utils.py
import rclpy
from rclpy.node import Node

import logging
logger = logging.getLogger('django')

class MyRos2Node(Node):
    def __init__(self):
        super().__init__('my_ros2_node')
        # Your ROS2 initialization code here
        self.get_logger().error("\n\nHERE\n\n")

_ros2_node_instance = None

def get_ros2_node_instance():
    global _ros2_node_instance
    if _ros2_node_instance is None:
        if not rclpy.ok():
            logger.info(f'\n\n{rclpy.ok()}\n\n')
            rclpy.init(args=None)
            _ros2_node_instance = MyRos2Node()
            logger.info(f'\n\n{_ros2_node_instance}\n\n')
    return _ros2_node_instance