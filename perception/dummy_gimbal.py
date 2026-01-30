#!/usr/bin/env python3

from mrover.srv import Dummy

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import time


class DummyServer(Node):
    def __init__(self):
        super().__init__('dummy_gimbal')

        # Pano Action Server
        self.start_pano = self.create_service(Dummy, 'dummy', self.dummy_callback)


    def dummy_callback(self, _, response):
        self.get_logger().info("Dummy request recieved, sleeping")
        self.get_clock().sleep_for(Duration(seconds=5))
        self.get_logger().info("Woken up")
        response.success = True
        return response
    

def main(args=None):
    rclpy.init(args=args)

    serv = DummyServer()

    rclpy.spin(serv)

    serv.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

