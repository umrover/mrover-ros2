#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rclpy.logging import get_logger

from std_msgs.msg import String
from test_infra import MRoverTesting

from mrover.msg import TestEvent

import importlib
import importlib.util

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.index = 0
        self.events = []
        self.event_subscriber = self.create_subscription(
            TestEvent,
            'test_events',
            self.event_callback,
            10)

        self.event_subscriber

    def timer_callback(self):
        if len(self.events) == 0:
            self.get_logger().info("No Events Loaded...")
        elif self.index < len(self.events) and self.events[self.index](self):
            self.index += 1

    def event_callback(self, msg: TestEvent):
        self.get_logger().info(f"Recieved function_name: {msg.function_name} module_spec: {msg.module_spec}")

        try:
            module = importlib.import_module(msg.module_spec)

            function = getattr(module, msg.function_name)

            self.events.append(function)
        except ImportError:
            get_logger('testing').error("Error importing the module...")
        except AttributeError:
            get_logger('testing').error("Error getting the functino attribute...")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TestNode()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

