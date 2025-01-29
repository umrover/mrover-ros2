#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool

SERVICE_TYPE = SetBool
SERVICE_NAME = '/science_enable_heater_b0'

class Service(Node):

    def __init__(self):
        super().__init__('debug_service')
        self.srv = self.create_service(SERVICE_TYPE, SERVICE_NAME, self.callback)

    def callback(self, request, response):
        response.success = True
        self.get_logger().info(f'Incoming request: {request}' )

        return response


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Service())
    rclpy.shutdown()

if __name__ == "__main__":
    main()