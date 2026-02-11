#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import argparse
from mrover.msg import ControllerState


class MockGimbalPublisher(Node):
    def __init__(self, rate: float):
        super().__init__("mock_gimbal_publisher")

        self.publisher = self.create_publisher(
            ControllerState, "/gimbal_controller_state", 10
        )

        self.pitch = 0.0
        self.yaw = 0.0

        self.timer = self.create_timer(1.0 / rate, self.publish_state)

        self.get_logger().info(
            f"Publishing mock gimbal state to /gimbal_controller_state at {rate} Hz"
        )
        self.get_logger().info("Initial position: pitch=0.0 rad, yaw=0.0 rad")

    def publish_state(self):
        msg = ControllerState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gimbal"
        msg.name = ["pitch", "yaw"]
        msg.state = ["Armed", "Armed"]
        msg.error = ["", ""]
        msg.position = [self.pitch, self.yaw]
        msg.velocity = [0.0, 0.0]
        msg.current = [0.0, 0.0]
        msg.limit_hit = [0, 0]

        self.publisher.publish(msg)

    def set_position(self, pitch: float, yaw: float):
        self.pitch = pitch
        self.yaw = yaw
        self.get_logger().info(
            f"Position updated: pitch={math.degrees(pitch):.1f} deg, yaw={math.degrees(yaw):.1f} deg"
        )


def main(args=None):
    parser = argparse.ArgumentParser(description="Publish mock gimbal controller state")
    parser.add_argument(
        "-r", "--rate", type=float, default=1.0, help="Publishing rate in Hz (default: 1.0)"
    )
    parser.add_argument(
        "--pitch", type=float, default=0.0, help="Initial pitch in degrees (default: 0.0)"
    )
    parser.add_argument(
        "--yaw", type=float, default=0.0, help="Initial yaw in degrees (default: 0.0)"
    )

    parsed_args = parser.parse_args()

    rclpy.init(args=args)

    try:
        publisher = MockGimbalPublisher(parsed_args.rate)
        publisher.set_position(
            math.radians(parsed_args.pitch), math.radians(parsed_args.yaw)
        )
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
