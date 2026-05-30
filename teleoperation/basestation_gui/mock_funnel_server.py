#!/usr/bin/env python3
"""
Mock ROS2 node for FunnelControls.vue development.

Publishes /sp_controller_state and serves the /sp_funnel_servo service,
so the real basestation backend can run normally and forward everything.

Run with:
    python3 mock_funnel_server.py

Requires a sourced ROS2 workspace with mrover messages built.
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header

from mrover.msg import ControllerState
from mrover.srv import ServoPosition

PUBLISH_HZ = 10
MOVE_SPEED_RAD_PER_SEC = 1.5

SITES_RAD = [3.1415, 2.0071, 1.1693, 0.0, 5.1138, 4.2586]


class MockFunnelNode(Node):
    def __init__(self):
        super().__init__('mock_funnel')

        self.position_rad: float = SITES_RAD[3]  # start at Trash
        self.target_rad: float = SITES_RAD[3]
        self.last_tick = time.monotonic()

        self.state_pub = self.create_publisher(
            ControllerState, '/sp_controller_state', qos_profile_sensor_data
        )
        self.srv = self.create_service(
            ServoPosition, '/sp_funnel_servo', self.handle_set_position
        )
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.tick)

        self.get_logger().info('Mock funnel node ready — publishing /sp_controller_state at 10 Hz')

    def tick(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now

        diff = self.target_rad - self.position_rad
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        step = MOVE_SPEED_RAD_PER_SEC * dt
        if abs(diff) <= step:
            self.position_rad = self.target_rad
        else:
            self.position_rad += math.copysign(step, diff)
            self.position_rad %= 2 * math.pi

        msg = ControllerState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.names = ['funnel']
        msg.states = ['']
        msg.errors = ['']
        msg.positions = [self.position_rad]
        msg.velocities = [0.0]
        msg.currents = [0.0]
        msg.limits_hit = [0]
        self.state_pub.publish(msg)

    def handle_set_position(
        self, request: ServoPosition.Request, response: ServoPosition.Response
    ) -> ServoPosition.Response:
        if 'funnel' not in request.names:
            response.at_tgts = [False] * len(request.names)
            return response

        idx = request.names.index('funnel')
        self.target_rad = float(request.positions[idx])
        self.get_logger().info(
            f'funnel target -> {self.target_rad:.4f} rad ({math.degrees(self.target_rad):.1f} deg)'
        )
        response.at_tgts = [False] * len(request.names)
        return response


def main():
    rclpy.init(args=sys.argv)
    node = MockFunnelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
