#!/usr/bin/env python3

"""
Detects if the rover is "stuck" - if it is commanded a nonzero velocity but the rover is neither moving nor turning.
At URC (MDRS) there is sometimes sandy terrain. It is possible the wheels will get stuck free spinning and the rover will not move.
This node only alerts, it does not actually perform recovery.
"""

__author__ = "Quintin Dwight"
__credits__ = ["Neha", "Emerson"]

import sys

import numpy as np

import message_filters
import rclpy
from geometry_msgs.msg import Twist
from mrover.msg import StateMachineStateUpdate
from nav_msgs.msg import Odometry
from rclpy import Parameter
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool

RECOVERY_STATE_NAME = "RecoveryState"


class StuckDetector(Node):
    cmd_vel: np.ndarray
    real_vel: np.ndarray
    last_data_update: Time
    last_trigger_time: Time | None

    def __init__(self) -> None:
        super().__init__("stuck_detector")

        self.get_logger().info("Starting...")

        self.declare_parameters(
            "",
            [
                ("window_size", Parameter.Type.INTEGER),
                ("angular_vel_thresh", Parameter.Type.DOUBLE),
                ("linear_vel_thresh", Parameter.Type.DOUBLE),
                ("post_recovery_grace_period", Parameter.Type.DOUBLE),
                ("data_watchdog_timeout", Parameter.Type.DOUBLE),
            ],
        )

        self.stuck_pub = self.create_publisher(Bool, "nav_stuck", 1)

        nav_status_sub = message_filters.Subscriber(self, StateMachineStateUpdate, "nav_state")
        odometry_sub = message_filters.Subscriber(self, Odometry, "odometry")
        cmd_vel_sub = message_filters.Subscriber(self, Twist, "cmd_vel")
        message_filters.ApproximateTimeSynchronizer(
            [nav_status_sub, cmd_vel_sub, odometry_sub], 10, 1.0, allow_headerless=True
        ).registerCallback(self.update)

        self.last_trigger_time = None

        self.reset()

        self.get_logger().info("Armed!")

    def reset(self) -> None:
        window_size = self.get_parameter("window_size").value

        self.cmd_vel = np.zeros((2, window_size))
        self.real_vel = np.zeros((2, window_size))
        self.last_data_update = self.get_clock().now()

    def update(self, nav_status: StateMachineStateUpdate, cmd_vel: Twist, odometry: Odometry) -> None:
        now = self.get_clock().now()

        post_recovery_grace_period = Duration(seconds=self.get_parameter("post_recovery_grace_period").value)
        angular_threshold = self.get_parameter("angular_vel_thresh").value
        linear_threshold = self.get_parameter("linear_vel_thresh").value

        data_watchdog_timeout = Duration(seconds=self.get_parameter("data_watchdog_timeout").value)
        if self.last_data_update and now - self.last_data_update > data_watchdog_timeout:
            self.get_logger().warn("Resetting failure identification due to long time since last update")
            self.reset()

        # Shift over all columns and replace the first column with the new values
        # This essentially advances forward te window
        self.cmd_vel = np.roll(self.cmd_vel, 1, axis=1)
        self.real_vel = np.roll(self.real_vel, 1, axis=1)
        self.cmd_vel[:, 0] = [cmd_vel.linear.x, cmd_vel.angular.z]
        self.real_vel[:, 0] = [
            odometry.twist.twist.linear.x,
            odometry.twist.twist.angular.z,
        ]

        _, window_size = self.cmd_vel.shape
        is_trying_to_move = np.count_nonzero(self.cmd_vel) >= window_size
        linear_speed_average = np.average(np.abs(self.real_vel[0, :]))
        angular_speed_average = np.average(np.abs(self.real_vel[1, :]))
        is_not_moving = linear_speed_average < linear_threshold and angular_speed_average < angular_threshold
        is_outside_grace = self.last_trigger_time is None or now - self.last_trigger_time > post_recovery_grace_period
        is_not_recovery = nav_status.state != RECOVERY_STATE_NAME

        if is_trying_to_move and is_not_moving and is_outside_grace and is_not_recovery:
            self.get_logger().warn("Detecting rover being stuck!")
            is_stuck = True
            self.last_trigger_time = now
        else:
            is_stuck = False

        self.stuck_pub.publish(is_stuck)

        self.last_data_update = now


def main() -> None:
    rclpy.init(args=sys.argv)
    try:
        rclpy.spin(StuckDetector())
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    main()
