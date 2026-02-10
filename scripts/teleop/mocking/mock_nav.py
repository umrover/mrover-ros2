#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import random
import argparse

from sensor_msgs.msg import NavSatFix
from mrover.msg import StateMachineStateUpdate, LED
import tf2_ros
from geometry_msgs.msg import TransformStamped

NAV_STATES = ["OffState", "WaypointState", "ApproachTargetState", "DoneState"]
LED_COLORS = [LED.RED, LED.BLUE, LED.BLINKING_GREEN, LED.RED]


class MockNav(Node):
    def __init__(self, rate: float, nav_cycle_sec: float):
        super().__init__("mock_nav")

        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.base_pub = self.create_publisher(NavSatFix, "basestation/position", 10)
        self.drone_pub = self.create_publisher(NavSatFix, "/drone_odometry", 10)
        self.nav_state_pub = self.create_publisher(StateMachineStateUpdate, "/nav_state", 10)
        self.led_pub = self.create_publisher(LED, "/led", 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.rover_lat = 42.293
        self.rover_lon = -83.713
        self.drone_angle = 0.0
        self.yaw = 0.0
        self.t = 0.0
        self.nav_cycle_sec = nav_cycle_sec
        self.nav_state_idx = 0

        self.create_timer(1.0 / rate, self.publish_gps)
        self.create_timer(0.1, self.publish_tf)
        self.create_timer(nav_cycle_sec, self.advance_nav_state)

        self.get_logger().info(f"Mock nav node started at {rate} Hz, nav cycle {nav_cycle_sec}s")

    def publish_gps(self):
        self.rover_lat += random.uniform(-0.00001, 0.00001)
        self.rover_lon += random.uniform(-0.00001, 0.00001)

        rover = NavSatFix()
        rover.header.stamp = self.get_clock().now().to_msg()
        rover.header.frame_id = "base_link"
        rover.latitude = self.rover_lat
        rover.longitude = self.rover_lon
        rover.altitude = 250.0
        self.gps_pub.publish(rover)

        base = NavSatFix()
        base.header.stamp = self.get_clock().now().to_msg()
        base.header.frame_id = "basestation"
        base.latitude = 42.294
        base.longitude = -83.714
        base.altitude = 250.0
        self.base_pub.publish(base)

        self.drone_angle += 0.02
        drone = NavSatFix()
        drone.header.stamp = self.get_clock().now().to_msg()
        drone.header.frame_id = "drone"
        drone.latitude = 42.295 + 0.001 * math.cos(self.drone_angle)
        drone.longitude = -83.712 + 0.001 * math.sin(self.drone_angle)
        drone.altitude = 270.0
        self.drone_pub.publish(drone)

    def publish_tf(self):
        self.yaw += 0.01

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2)
        t.transform.rotation.w = math.cos(self.yaw / 2)
        self.tf_broadcaster.sendTransform(t)

    def advance_nav_state(self):
        state_msg = StateMachineStateUpdate()
        state_msg.state_machine_name = "NavigationStateMachine"
        state_msg.state = NAV_STATES[self.nav_state_idx]
        self.nav_state_pub.publish(state_msg)

        led_msg = LED()
        led_msg.color = LED_COLORS[self.nav_state_idx]
        self.led_pub.publish(led_msg)

        self.get_logger().info(f"Nav state: {NAV_STATES[self.nav_state_idx]}")
        self.nav_state_idx = (self.nav_state_idx + 1) % len(NAV_STATES)


def main(args=None):
    parser = argparse.ArgumentParser(description="Mock navigation subsystem")
    parser.add_argument("-r", "--rate", type=float, default=5.0, help="GPS publishing rate in Hz (default: 5.0)")
    parser.add_argument("--nav-cycle-sec", type=float, default=15.0, help="Nav state cycle period in seconds (default: 15.0)")
    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    try:
        node = MockNav(parsed_args.rate, parsed_args.nav_cycle_sec)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
