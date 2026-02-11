#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import argparse

from mrover.msg import ControllerState, Throttle, IK
from mrover.srv import IkMode
from geometry_msgs.msg import Twist, Vector3

ARM_JOINTS = ["joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "gripper", "cam"]
IK_MODE_NAMES = {0: "POSITION_CONTROL", 1: "VELOCITY_CONTROL", 2: "TYPING"}


class MockArm(Node):
    def __init__(self, rate: float):
        super().__init__("mock_arm")

        self.state_pub = self.create_publisher(ControllerState, "/arm_controller_state", 10)
        self.ik_pub = self.create_publisher(IK, "/arm_ik", 10)

        self.create_subscription(Throttle, "/arm_thr_cmd", self.on_throttle, 10)
        self.create_subscription(IK, "/ik_pos_cmd", self.on_ik_pos, 10)
        self.create_subscription(Twist, "/ik_vel_cmd", self.on_ik_vel, 10)

        self.create_service(IkMode, "/ik_mode", self.on_ik_mode)

        self.t = 0.0
        self.last_log_time = 0.0
        self.timer = self.create_timer(1.0 / rate, self.publish_state)

        self.get_logger().info(f"Mock arm node started at {rate} Hz")

    def publish_state(self):
        self.t += 0.01

        state = ControllerState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = "arm"
        state.names = ARM_JOINTS
        state.states = ["Armed"] * len(ARM_JOINTS)
        state.errors = [""] * len(ARM_JOINTS)
        state.positions = [math.sin(self.t * (0.3 + i * 0.1)) * 1.5 for i in range(len(ARM_JOINTS))]
        state.velocities = [0.0] * len(ARM_JOINTS)
        state.currents = [0.05 + 0.03 * i for i in range(len(ARM_JOINTS))]
        state.limits_hit = [0] * len(ARM_JOINTS)
        self.state_pub.publish(state)

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
            self.get_logger().info(f"IK pos: ({msg.pos.x:.3f}, {msg.pos.y:.3f}, {msg.pos.z:.3f}) pitch={msg.pitch:.2f} roll={msg.roll:.2f}")
            self.last_log_time = now

    def on_ik_vel(self, msg: Twist):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log_time > 1.0:
            self.get_logger().info(f"IK vel: linear=({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f})")
            self.last_log_time = now

    def on_ik_mode(self, request, response):
        mode_name = IK_MODE_NAMES.get(request.mode, f"UNKNOWN({request.mode})")
        self.get_logger().info(f"IK mode set to {mode_name}")
        response.success = True
        return response


def main(args=None):
    parser = argparse.ArgumentParser(description="Mock arm subsystem")
    parser.add_argument("-r", "--rate", type=float, default=10.0, help="Publishing rate in Hz (default: 10.0)")
    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    try:
        node = MockArm(parsed_args.rate)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
