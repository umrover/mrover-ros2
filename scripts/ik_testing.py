#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist


class CirclePublisher(Node):
    def __init__(self):
        super().__init__("ik_circle_publisher")
        self.vel_pub = self.create_publisher(Twist, "ee_vel_cmd", 10)
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.time = 0
        self.radius = 0.2  # meters
        self.period = 5  # seconds

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = (2 * math.pi / self.period) * self.radius * math.cos(2 * math.pi * self.time / self.period)
        msg.linear.z = -1 * (2 * math.pi / self.period) * self.radius * math.sin(2 * math.pi * self.time / self.period)
        self.vel_pub.publish(msg)
        self.time += self.dt


class MPublisher(Node):
    def __init__(self):
        super().__init__("ik_m_publisher")
        self.vel_pub = self.create_publisher(Twist, "ee_vel_cmd", 10)
        self.dt = 0.1
        self.size = 0.4  # size of M in meters
        self.speed = 0.2
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.time = 0

    def timer_callback(self):
        msg = Twist()
        distance_traveled = self.time * self.speed
        if distance_traveled < self.size:
            msg.linear.z = self.speed
        elif distance_traveled < self.size * (1 + math.sqrt(2) / 2):
            msg.linear.z = self.speed * (-math.sqrt(2) / 2)
            msg.linear.y = self.speed * (-math.sqrt(2) / 2)
        elif distance_traveled < self.size * (1 + math.sqrt(2)):
            msg.linear.z = self.speed * (math.sqrt(2) / 2)
            msg.linear.y = self.speed * (-math.sqrt(2) / 2)
        elif distance_traveled < self.size * (2 + math.sqrt(2)):
            msg.linear.z = -self.speed
        else:
            print("Done!")
            sys.exit(0)
        self.vel_pub.publish(msg)
        self.time += self.dt


if __name__ == "__main__":
    try:
        rclpy.init(args=sys.argv)
        rclpy.spin(MPublisher())
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
