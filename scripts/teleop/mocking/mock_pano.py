#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import argparse
import numpy as np

from mrover.srv import PanoramaStart, PanoramaEnd
from sensor_msgs.msg import Image


class MockPano(Node):
    def __init__(self):
        super().__init__("mock_pano")

        self.create_service(PanoramaStart, "/panorama/start", self.on_start)
        self.create_service(PanoramaEnd, "/panorama/end", self.on_end)

        self.get_logger().info("Mock panorama node started")

    def on_start(self, request, response):
        self.get_logger().info("Panorama started")
        return response

    def on_end(self, request, response):
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


def main(args=None):
    parser = argparse.ArgumentParser(description="Mock panorama subsystem")
    parser.parse_args()

    rclpy.init(args=args)
    try:
        node = MockPano()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
