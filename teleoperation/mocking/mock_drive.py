#!/usr/bin/env python3

"""
Mock drive controller state for DriveDataTable.vue.

Exercises: sign changes on velocity, magnitude swings on current,
left/right phase offset, error cycling, stale periods.
"""

import math
import argparse

import rclpy
from rclpy.node import Node

from mrover.msg import ControllerState
from geometry_msgs.msg import Twist

LEFT_MOTORS = ["front_left", "middle_left", "back_left"]
RIGHT_MOTORS = ["front_right", "middle_right", "back_right"]
STATES_CYCLE = ["Armed", "Armed", "Disarmed"]
ERRORS_CYCLE = ["", "", "WatchdogTimeout"]


def oscillate(t: float, freq: float, amplitude: float, phase: float = 0.0) -> float:
    return amplitude * math.sin(t * freq + phase)


class MockDrive(Node):
    def __init__(self, rate: float, stale_period: float, stale_duration: float):
        super().__init__("mock_drive")

        self.left_pub = self.create_publisher(ControllerState, "/left_controller_state", 10)
        self.right_pub = self.create_publisher(ControllerState, "/right_controller_state", 10)

        self.create_subscription(Twist, "/joystick_vel_cmd", self.on_joystick, 10)
        self.create_subscription(Twist, "/controller_vel_cmd", self.on_controller, 10)

        self.t = 0.0
        self.dt = 1.0 / rate
        self.stale_period = stale_period
        self.stale_duration = stale_duration
        self.last_log_time = 0.0

        self.create_timer(self.dt, self.tick)
        self.get_logger().info(f"Mock drive started at {rate} Hz")

    def in_stale(self) -> bool:
        if self.stale_period <= 0:
            return False
        return (self.t % self.stale_period) > (self.stale_period - self.stale_duration)

    def make_state(self, names: list[str], phase: float) -> ControllerState:
        stamp = self.get_clock().now().to_msg()
        n = len(names)
        state_idx = int(self.t / 10.0) % len(STATES_CYCLE)
        error_idx = int(self.t / 15.0) % len(ERRORS_CYCLE)

        msg = ControllerState()
        msg.header.stamp = stamp
        msg.header.frame_id = "drive"
        msg.names = list(names)
        msg.states = [STATES_CYCLE[(state_idx + i) % len(STATES_CYCLE)] for i in range(n)]
        msg.errors = [ERRORS_CYCLE[(error_idx + i) % len(ERRORS_CYCLE)] for i in range(n)]
        msg.positions = [oscillate(self.t, 0.2 + i * 0.05, 9.0, phase=phase + i) for i in range(n)]
        msg.velocities = [oscillate(self.t, 0.5 + i * 0.1, 8.0, phase=phase + i * 0.7) for i in range(n)]
        msg.currents = [oscillate(self.t, 0.3 + i * 0.08, 4.0, phase=phase + i * 1.1) for i in range(n)]
        msg.limits_hit = [0] * n
        return msg

    def tick(self):
        self.t += self.dt
        if self.in_stale():
            return
        self.left_pub.publish(self.make_state(LEFT_MOTORS, 0.0))
        self.right_pub.publish(self.make_state(RIGHT_MOTORS, 2.0))

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
    parser = argparse.ArgumentParser(description="Mock drive controller state")
    parser.add_argument("-r", "--rate", type=float, default=10.0, help="Publish rate in Hz (default: 10)")
    parser.add_argument("--stale-period", type=float, default=45.0, help="Stale cycle in seconds, 0 to disable (default: 45)")
    parser.add_argument("--stale-duration", type=float, default=3.0, help="Stale length in seconds (default: 3)")
    parsed = parser.parse_args()

    rclpy.init(args=args)
    try:
        rclpy.spin(MockDrive(parsed.rate, parsed.stale_period, parsed.stale_duration))
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
