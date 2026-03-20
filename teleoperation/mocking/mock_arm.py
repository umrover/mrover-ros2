#!/usr/bin/env python3

"""
Mock arm controller state for ArmDataTable.vue.

Exercises: sign changes, single-digit values with zero crossings,
error/state cycling, limit hit toggling, stale periods (no publish).
"""

import math
import argparse

import rclpy
from rclpy.node import Node

from mrover.msg import ControllerState, Throttle, IK
from mrover.srv import IkMode
from geometry_msgs.msg import Twist, Vector3

ARM_JOINTS = ["pusher", "gripper", "joint_de_roll", "joint_de_pitch", "joint_c", "joint_b", "joint_a"]
STATES_CYCLE = ["Armed", "Armed", "Armed", "Disarmed"]
ERRORS_CYCLE = ["", "", "", "OverTemp"]


def oscillate(t: float, freq: float, amplitude: float, phase: float = 0.0) -> float:
    return amplitude * math.sin(t * freq + phase)


def square_wave(t: float, period: float, duty: float) -> bool:
    return (t % period) < duty


class MockArm(Node):
    def __init__(self, rate: float, stale_period: float, stale_duration: float):
        super().__init__("mock_arm")

        self.state_pub = self.create_publisher(ControllerState, "/arm_controller_state", 10)
        self.ik_pub = self.create_publisher(IK, "/arm_ik", 10)

        self.create_subscription(Throttle, "/arm_thr_cmd", self.on_throttle, 10)
        self.create_subscription(IK, "/ik_pos_cmd", self.on_ik_pos, 10)
        self.create_subscription(Twist, "/ik_vel_cmd", self.on_ik_vel, 10)
        self.create_service(IkMode, "/ik_mode", self.on_ik_mode)

        self.t = 0.0
        self.dt = 1.0 / rate
        self.stale_period = stale_period
        self.stale_duration = stale_duration
        self.last_log_time = 0.0

        self.create_timer(self.dt, self.tick)
        self.get_logger().info(f"Mock arm started at {rate} Hz")

    def in_stale(self) -> bool:
        if self.stale_period <= 0:
            return False
        return (self.t % self.stale_period) > (self.stale_period - self.stale_duration)

    def tick(self):
        self.t += self.dt
        if self.in_stale():
            return

        stamp = self.get_clock().now().to_msg()
        n = len(ARM_JOINTS)
        state_idx = int(self.t / 8.0) % len(STATES_CYCLE)
        error_idx = int(self.t / 10.0) % len(ERRORS_CYCLE)

        msg = ControllerState()
        msg.header.stamp = stamp
        msg.header.frame_id = "arm"
        msg.names = list(ARM_JOINTS)
        msg.states = [STATES_CYCLE[(state_idx + i) % len(STATES_CYCLE)] for i in range(n)]
        msg.errors = [ERRORS_CYCLE[(error_idx + i) % len(ERRORS_CYCLE)] for i in range(n)]
        msg.positions = [oscillate(self.t, 0.3 + i * 0.05, 9.0, phase=i * 0.8) for i in range(n)]
        msg.velocities = [oscillate(self.t, 0.5 + i * 0.07, 5.0, phase=i * 0.8) for i in range(n)]
        msg.currents = [oscillate(self.t, 0.4 + i * 0.06, 3.0, phase=i * 1.2) for i in range(n)]
        msg.limits_hit = [1 if square_wave(self.t + i * 3.0, 12.0, 2.0) else 0 for i in range(n)]
        self.state_pub.publish(msg)

        ik = IK()
        ik.pos = Vector3(x=0.3, y=0.0, z=0.2 + 0.05 * math.sin(self.t * 0.5))
        ik.pitch = 0.0
        ik.roll = 0.0
        self.ik_pub.publish(ik)

    def on_throttle(self, msg: Throttle):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log_time > 1.0:
            pairs = ", ".join(f"{n}={t:.2f}" for n, t in zip(msg.names, msg.throttles))
            self.get_logger().info(f"Arm throttle: {pairs}")
            self.last_log_time = now

    def on_ik_pos(self, msg: IK):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log_time > 1.0:
            self.get_logger().info(f"IK pos: ({msg.pos.x:.3f}, {msg.pos.y:.3f}, {msg.pos.z:.3f})")
            self.last_log_time = now

    def on_ik_vel(self, msg: Twist):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log_time > 1.0:
            self.get_logger().info(f"IK vel: ({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f})")
            self.last_log_time = now

    def on_ik_mode(self, request, response):
        self.get_logger().info(f"IK mode set to {request.mode}")
        response.success = True
        return response


def main(args=None):
    parser = argparse.ArgumentParser(description="Mock arm controller state")
    parser.add_argument("-r", "--rate", type=float, default=10.0, help="Publish rate in Hz (default: 10)")
    parser.add_argument("--stale-period", type=float, default=45.0, help="Stale cycle in seconds, 0 to disable (default: 45)")
    parser.add_argument("--stale-duration", type=float, default=3.0, help="Stale length in seconds (default: 3)")
    parsed = parser.parse_args()

    rclpy.init(args=args)
    try:
        rclpy.spin(MockArm(parsed.rate, parsed.stale_period, parsed.stale_duration))
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
