#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import service
from std_srvs.srv import SetBool
from mrover.srv import ServoPosition

SERVICE_TYPE = ServoPosition
SERVICE_NAME = "/sp_funnel_servo"


class Service(Node):

    def __init__(self):
        super().__init__("debug_service")
        self.get_logger().info(f"Debugging: {SERVICE_NAME}")
        self.srv = self.create_service(SERVICE_TYPE, SERVICE_NAME, self.callback)

    def callback(self, request, response):
        self.get_logger().info(f"Incoming request: {request}")
        return response


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Service())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
