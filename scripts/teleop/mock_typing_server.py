#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from mrover.action import TypingCode

TYPING_DELAY_SEC = 1.0


class MockTypingServer(Node):
    def __init__(self):
        super().__init__("mock_typing_server")
        self.action_server = ActionServer(self, TypingCode, "/es_typing_code", self.execute_callback)
        self.get_logger().info("Mock typing server started on /es_typing_code")

    def execute_callback(self, goal_handle):
        code = goal_handle.request.launch_code
        self.get_logger().info(f"Received goal: {code}")

        feedback = TypingCode.Feedback()

        for i, char in enumerate(code):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal cancelled")
                return TypingCode.Result()

            feedback.current_index = i
            feedback.current_state = "in_progress"
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"Typing character {i + 1}/{len(code)}: {char}")
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=TYPING_DELAY_SEC))

        feedback.current_index = len(code)
        feedback.current_state = "complete"
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        self.get_logger().info("Goal succeeded")
        return TypingCode.Result()


def main():
    rclpy.init()
    node = MockTypingServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
