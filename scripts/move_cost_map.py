#!/usr/bin/env python3

import sys
import time

import numpy as np

import rclpy
import tf2_ros
from rclpy.node import Node
from mrover.srv import MoveCostMap
from lie.conversions import SE3, from_position_orientation, from_tf_tree
from lie.lie import SE3Conversions

class MoveCostMapNode(Node):
    def __init__(self) -> None:
        super().__init__("move_cost_map_node")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(1, self.move_request)

    def move_request(self):
        self.move_cost_map_srv = self.create_client(MoveCostMap, 'move_cost_map')

        while not self.move_cost_map_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        position = SE3.from_tf_tree(self.tf_buffer, 'base_link', 'map')
        tx, ty, _ = position.translation()
        course_in_map = from_position_orientation(tx, ty)

        SE3.to_tf_tree(self.tf_broadcaster, course_in_map, "debug_course", "map")

        self.req = MoveCostMap.Request()

        self.req.course = "debug_course"

        self.future = self.move_cost_map_srv.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        if(self.future.result().success):
            self.get_logger().info("Successfuly Sent Moved Cost Map")


def main() -> None:
    rclpy.init(args=sys.argv)
    node = MoveCostMapNode()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
