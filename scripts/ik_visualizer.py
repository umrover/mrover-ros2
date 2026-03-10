#!/usr/bin/env python3

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.client import Client
from rclpy.executors import ExternalShutdownException

class IK_Visualization():
    def __init__(self):
        return


def main():
    try:
        rclpy.init(args=sys.argv)
        rclpy.spin_once(IK_Visualization())
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    main()
