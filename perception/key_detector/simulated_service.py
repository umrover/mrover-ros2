#!/usr/bin/env python3

from srv import GetKeyLoc

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(GetKeyLoc, 'GetKeyLoc', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.x = 3
        response.y = 3
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.char))

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()