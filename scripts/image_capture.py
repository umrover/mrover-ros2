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
    def __init__(self) -> None:
        super().__init__("image_capture")

        self.image_sub = self.create_subscription(Image, "/zed/left/image", self.save_image, 1)

    def save_image(self, msg: Image):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        unique_id = "{date:%Y-%m-%d_%H:%M:%S}".format(date=datetime.datetime.now())

        path = (
            Path(get_package_share_directory("mrover"))
            / ".."
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
    try:
        rclpy.init(args=sys.argv)

        imgNode = ImageCapture()
        amount = 0

        while True:
            input()
            amount += 1
            print(f"Amount {amount}")
            rclpy.spin_once(imgNode)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    main()
