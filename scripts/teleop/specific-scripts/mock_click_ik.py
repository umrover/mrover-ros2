#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from mrover.action import ClickIk


class MockClickIkServer(Node):
    def __init__(self):
        super().__init__("mock_click_ik_server")
        self.action_server = ActionServer(self, ClickIk, "/click_ik", self.execute_callback)
        self.get_logger().info("Mock ClickIk action server ready on /click_ik")

    def execute_callback(self, goal_handle):
        x = goal_handle.request.point_in_image_x
        y = goal_handle.request.point_in_image_y
        self.get_logger().info(f"Received ClickIk goal: ({x}, {y})")

        feedback = ClickIk.Feedback()
        for dist in range(100, 0, -1):
            feedback.distance = float(dist)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)

        goal_handle.succeed()
        result = ClickIk.Result()
        result.success = True
        self.get_logger().info("ClickIk goal succeeded")
        return result


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MockClickIkServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
