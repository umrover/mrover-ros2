#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg._image import Image
from ament_index_python import get_package_share_directory

import numpy as np
import sys
from pathlib import Path
import datetime

import cv2

from rclpy.executors import ExternalShutdownException


class ImageCapture(Node):
    def __init__(self, topic) -> None:
        super().__init__("image_capture")

        self.image_sub = self.create_subscription(Image, topic, self.save_image, 1)
        self.get_logger().info(f"Creating a subscriber on {topic}")

    def save_image(self, msg: Image):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        unique_id = "{date:%Y-%m-%d_%H:%M:%S}".format(date=datetime.datetime.now())

        path = (
            Path(get_package_share_directory("mrover"))
            / ".."
            / ".."
            / ".."
            / ".."
            / "src"
            / "mrover"
            / "data"
            / "images"
        )

        if not path.exists():
            path.mkdir(parents=True, exist_ok=True)

        path = path / f"image_{unique_id}.jpg"

        cv2.imwrite(str(path), img)

        self.get_logger().info(f"Saved image_{unique_id}.jpg")
        pass


def main() -> None:
    args = sys.argv
    argc = len(sys.argv)
    
    if argc != 2:
        print(f'Usage: ros2 run mrover image_capture.py [ros2 image topic]')
        sys.exit(1)

    topic = sys.argv[1]
    print(f'Recieved topic as {topic}')

    try:
        rclpy.init(args=sys.argv)
        while True:
            input()
            node = ImageCapture(topic)
            rclpy.spin_once(node)
            node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    main()
