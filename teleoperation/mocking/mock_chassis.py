#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import argparse
import numpy as np

from mrover.msg import ControllerState
from mrover.srv import PanoramaStart, PanoramaEnd, ServoPosition
from sensor_msgs.msg import Image

GIMBAL_MOTORS = ["mast_gimbal_pitch", "mast_gimbal_yaw"]


class MockChassis(Node):
    def __init__(self, rate: float):
        super().__init__("mock_chassis")

        self.state_pub = self.create_publisher(ControllerState, "/gimbal_controller_state", 10)

        self.create_service(PanoramaStart, "/panorama/start", self.on_panorama_start)
        self.create_service(PanoramaEnd, "/panorama/end", self.on_panorama_end)
        self.create_service(ServoPosition, "/gimbal_servo", self.on_gimbal_servo)
        self.create_service(ServoPosition, "/sp_funnel_servo", self.on_funnel_servo)

        self.pitch = 0.0
        self.yaw = 0.0
        self.timer = self.create_timer(1.0 / rate, self.publish_state)

        self.get_logger().info(f"Mock chassis node started at {rate} Hz")

    def publish_state(self):
        msg = ControllerState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gimbal"
        msg.names = GIMBAL_MOTORS
        msg.states = ["Armed", "Armed"]
        msg.errors = ["", ""]
        msg.positions = [self.pitch, self.yaw]
        msg.velocities = [0.0, 0.0]
        msg.currents = [0.0, 0.0]
        msg.limits_hit = [0, 0]
        self.state_pub.publish(msg)

    def on_panorama_start(self, request, response):
        self.get_logger().info("Panorama started")
        return response

    def on_panorama_end(self, request, response):
        self.get_logger().info("Panorama ended, generating dummy image")
        img = Image()
        img.height = 100
        img.width = 400
        img.encoding = "rgba8"
        img.step = 400 * 4
        data = np.zeros((100, 400, 4), dtype=np.uint8)
        data[:, :, 0] = np.linspace(0, 255, 400, dtype=np.uint8)
        data[:, :, 3] = 255
        img.data = data.tobytes()
        response.success = True
        response.img = img
        return response

    def on_gimbal_servo(self, request, response):
        for i, name in enumerate(request.names):
            if name == "mast_gimbal_pitch":
                self.pitch = request.positions[i]
            elif name == "mast_gimbal_yaw":
                self.yaw = request.positions[i]
        self.get_logger().info(f"Gimbal servo: pitch={math.degrees(self.pitch):.1f} yaw={math.degrees(self.yaw):.1f}")
        response.at_tgts = [True] * len(request.names)
        return response

    def on_funnel_servo(self, request, response):
        pairs = ", ".join(f"{n}={p:.2f}" for n, p in zip(request.names, request.positions))
        self.get_logger().info(f"Funnel servo: {pairs}")
        response.at_tgts = [True] * len(request.names)
        return response


def main(args=None):
    parser = argparse.ArgumentParser(description="Mock chassis subsystem")
    parser.add_argument("-r", "--rate", type=float, default=5.0, help="Publishing rate in Hz (default: 5.0)")
    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    try:
        node = MockChassis(parsed_args.rate)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
