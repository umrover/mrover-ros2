#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import argparse

from mrover.msg import ControllerState
from mrover.srv import ServoPosition

JOINTS = ["pitch", "yaw"]


class MockGimbal(Node):
    def __init__(self, rate: float):
        super().__init__("mock_gimbal")

        self.state_pub = self.create_publisher(ControllerState, "/gimbal_controller_state", 10)
        self.create_service(ServoPosition, "/gimbal_servo", self.on_servo_request)

        self.pitch = 0.0
        self.yaw = 0.0
        self.timer = self.create_timer(1.0 / rate, self.publish_state)

        self.get_logger().info(f"Mock gimbal node started at {rate} Hz")

    def publish_state(self):
        msg = ControllerState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gimbal"
        msg.names = JOINTS
        msg.states = ["Armed", "Armed"]
        msg.errors = ["", ""]
        msg.positions = [self.pitch, self.yaw]
        msg.velocities = [0.0, 0.0]
        msg.currents = [0.0, 0.0]
        msg.limits_hit = [0, 0]
        self.state_pub.publish(msg)

    def on_servo_request(self, request, response):
        for i, name in enumerate(request.names):
            if name == "pitch":
                self.pitch = request.positions[i]
            elif name == "yaw":
                self.yaw = request.positions[i]
        self.get_logger().info(
            f"Gimbal servo: pitch={math.degrees(self.pitch):.1f} yaw={math.degrees(self.yaw):.1f}"
        )
        response.at_tgts = [True] * len(request.names)
        return response


def main(args=None):
    parser = argparse.ArgumentParser(description="Mock gimbal subsystem")
    parser.add_argument("-r", "--rate", type=float, default=5.0, help="State publishing rate in Hz (default: 5.0)")
    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    try:
        node = MockGimbal(parsed_args.rate)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
