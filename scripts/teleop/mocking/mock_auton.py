#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import argparse

from mrover.action import TypingCode
from mrover.srv import EnableAuton
from std_srvs.srv import SetBool


class MockAuton(Node):
    def __init__(self, typing_delay: float):
        super().__init__("mock_auton")

        self.typing_delay = typing_delay

        self.create_service(EnableAuton, "/enable_auton", self.on_enable_auton)
        self.create_service(SetBool, "/toggle_pure_pursuit", self.on_toggle("pure_pursuit"))
        self.create_service(SetBool, "/toggle_path_relaxation", self.on_toggle("path_relaxation"))
        self.create_service(SetBool, "/toggle_path_interpolation", self.on_toggle("path_interpolation"))

        self.action_server = ActionServer(self, TypingCode, "/es_typing_code", self.execute_typing)

        self.get_logger().info(f"Mock auton node started (typing delay: {typing_delay}s)")

    def on_enable_auton(self, request, response):
        self.get_logger().info(f"Auton {'enabled' if request.enable else 'disabled'}, {len(request.waypoints)} waypoints")
        response.success = True
        return response

    def on_toggle(self, name: str):
        def handler(request, response):
            self.get_logger().info(f"{name} {'enabled' if request.data else 'disabled'}")
            response.success = True
            response.message = f"{name} toggled"
            return response
        return handler

    def execute_typing(self, goal_handle):
        code = goal_handle.request.launch_code
        self.get_logger().info(f"Typing goal received: {code}")

        feedback = TypingCode.Feedback()

        for i, char in enumerate(code):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Typing goal cancelled")
                return TypingCode.Result()

            feedback.current_index = i
            feedback.current_state = "in_progress"
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"Typing {i + 1}/{len(code)}: '{char}'")
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=self.typing_delay))

        feedback.current_index = len(code)
        feedback.current_state = "complete"
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        self.get_logger().info("Typing goal succeeded")
        return TypingCode.Result()


def main(args=None):
    parser = argparse.ArgumentParser(description="Mock autonomy subsystem")
    parser.add_argument("--typing-delay", type=float, default=1.0, help="Delay between typed characters in seconds (default: 1.0)")
    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    try:
        node = MockAuton(parsed_args.typing_delay)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
