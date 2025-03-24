#!/usr/bin/env python3

import sys
from dataclasses import dataclass, field
from functools import partial

import rclpy
from geometry_msgs.msg import Twist
from rclpy import Parameter
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.time import Time


class Superstructure(Node):
    @dataclass
    class Publisher:
        sub: Subscription
        twist: Twist = field(default_factory=Twist)
        last_time: Time | None = None

    def __init__(self) -> None:
        super().__init__("superstructure")

        self.declare_parameters(
            "",
            [
                ("input_topics", Parameter.Type.STRING_ARRAY),
                ("timeout", Parameter.Type.DOUBLE),
                ("rate", Parameter.Type.DOUBLE),
            ],
        )
        input_topics = self.get_parameter("input_topics").value

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.subs = {
            topic: self.Publisher(self.create_subscription(Twist, topic, partial(self.velocity_callback, topic), 10))
            for topic in input_topics
        }

        self.create_timer(1 / self.get_parameter("rate").value, self.timer_callback)

    def velocity_callback(self, topic: str, msg: Twist) -> None:
        self.subs[topic].twist = msg
        self.subs[topic].last_time = self.get_clock().now()

    def timer_callback(self) -> None:
        timeout = Duration(seconds=self.get_parameter("timeout").value)

        final = Twist()
        for sub in self.subs.values():
            if sub.last_time is None or (self.get_clock().now() - sub.last_time) > timeout:
                continue

            final.linear.x += sub.twist.linear.x
            final.angular.z += sub.twist.angular.z
        self.cmd_vel_pub.publish(final)


def main(args=None) -> None:
    try:
        rclpy.init(args=args)
        rclpy.spin(Superstructure())
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    main()
