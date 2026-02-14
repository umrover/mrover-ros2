#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import argparse

from mrover.msg import ControllerState
from geometry_msgs.msg import Twist

LEFT_MOTORS = ["front_left", "middle_left", "back_left"]
RIGHT_MOTORS = ["front_right", "middle_right", "back_right"]


class MockDrive(Node):
    def __init__(self, rate: float):
        super().__init__("mock_drive")

        self.left_pub = self.create_publisher(ControllerState, "/left_controller_state", 10)
        self.right_pub = self.create_publisher(ControllerState, "/right_controller_state", 10)

        self.create_subscription(Twist, "/joystick_vel_cmd", self.on_joystick, 10)
        self.create_subscription(Twist, "/controller_vel_cmd", self.on_controller, 10)

        self.t = 0.0
        self.last_log_time = 0.0
        self.timer = self.create_timer(1.0 / rate, self.publish_state)

        self.get_logger().info(f"Mock drive node started at {rate} Hz")

    def make_controller_state(self, names: list[str], phase_offset: float) -> ControllerState:
        msg = ControllerState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "drive"
        msg.names = names
        msg.states = ["Armed"] * len(names)
        msg.errors = [""] * len(names)
        msg.positions = [math.sin(self.t + i * phase_offset) * 10.0 for i in range(len(names))]
        msg.velocities = [0.0] * len(names)
        msg.currents = [0.1] * len(names)
        msg.limits_hit = [0] * len(names)
        return msg

    def publish_state(self):
        self.t += 0.01
        self.left_pub.publish(self.make_controller_state(LEFT_MOTORS, 0.5))
        self.right_pub.publish(self.make_controller_state(RIGHT_MOTORS, 0.7))

    def on_joystick(self, msg: Twist):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log_time > 1.0:
            self.get_logger().info(f"Joystick: linear={msg.linear.x:.2f} angular={msg.angular.z:.2f}")
            self.last_log_time = now

    def on_controller(self, msg: Twist):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log_time > 1.0:
            self.get_logger().info(f"Controller: linear={msg.linear.x:.2f} angular={msg.angular.z:.2f}")
            self.last_log_time = now


def main(args=None):
    parser = argparse.ArgumentParser(description="Mock drive subsystem")
    parser.add_argument("-r", "--rate", type=float, default=10.0, help="Publishing rate in Hz (default: 10.0)")
    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    try:
        node = MockDrive(parsed_args.rate)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
