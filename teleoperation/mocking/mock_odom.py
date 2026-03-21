#!/usr/bin/env python3

"""
Mock publisher for all ROS topics consumed by OdometryReading.vue.

Exercises: digit-count swings (1-3 digits), N/S and E/W sign flips,
GPS status cycling, IMU calibration toggling, velocity zero crossings,
and periodic blackout windows (no messages -> null state on frontend).
"""

import math
import argparse

import rclpy
from rclpy.node import Node
import tf2_ros
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import Twist, TransformStamped
from mrover.msg import CalibrationStatus

GPS_STATUSES = [
    (NavSatStatus.STATUS_NO_FIX, "NO_FIX"),
    (NavSatStatus.STATUS_FIX, "FIX"),
    (NavSatStatus.STATUS_SBAS_FIX, "SBAS"),
    (NavSatStatus.STATUS_GBAS_FIX, "GBAS"),
]


def make_navsatfix(stamp, frame_id: str, lat: float, lon: float, status: int, alt: float = 0.0) -> NavSatFix:
    msg = NavSatFix()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.status.status = status
    msg.latitude = lat
    msg.longitude = lon
    msg.altitude = alt
    return msg


def euler_to_quaternion(yaw: float, pitch: float, roll: float) -> tuple[float, float, float, float]:
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def swing_magnitude(t: float, phase: float = 0.0) -> float:
    """Oscillate between ~2 and ~142 to test 1-digit and 3-digit padding."""
    return 70.0 * math.sin(t * 1.5 + phase) + 72.0


def square_wave(t: float, period: float, duty: float) -> bool:
    """True for the first `duty` seconds of each `period`."""
    return (t % period) < duty


class MockOdom(Node):
    def __init__(self, rate: float, blackout_period: float, blackout_duration: float):
        super().__init__("mock_odom")

        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.base_pub = self.create_publisher(NavSatFix, "basestation/position", 10)
        self.drone_pub = self.create_publisher(NavSatFix, "/drone_odom", 10)
        self.cal_pub = self.create_publisher(CalibrationStatus, "/calibration", 10)
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.t = 0.0
        self.dt = 1.0 / rate
        self.blackout_period = blackout_period
        self.blackout_duration = blackout_duration
        self.gps_status_idx = 0

        self.create_timer(self.dt, self.tick)
        self.create_timer(5.0, self.cycle_gps_status)

        self.get_logger().info(f"Started at {rate} Hz (blackout: {blackout_duration}s every {blackout_period}s)")

    def in_blackout(self) -> bool:
        if self.blackout_period <= 0:
            return False
        return (self.t % self.blackout_period) > (self.blackout_period - self.blackout_duration)

    def cycle_gps_status(self):
        self.gps_status_idx = (self.gps_status_idx + 1) % len(GPS_STATUSES)
        self.get_logger().info(f"GPS status -> {GPS_STATUSES[self.gps_status_idx][1]}")

    def tick(self):
        self.t += self.dt
        if self.in_blackout():
            return

        stamp = self.get_clock().now().to_msg()
        self.publish_gps(stamp)
        self.publish_tf(stamp)
        self.publish_calibration(stamp)
        self.publish_velocity(stamp)

    def publish_gps(self, stamp):
        rover_swing = swing_magnitude(self.t)
        rover_status = GPS_STATUSES[self.gps_status_idx][0]
        self.gps_pub.publish(
            make_navsatfix(
                stamp,
                "base_link",
                lat=rover_swing * math.sin(self.t * 0.3),
                lon=rover_swing * math.cos(self.t * 0.2),
                status=rover_status,
                alt=250.0 + 5.0 * math.sin(self.t * 0.1),
            )
        )

        base_swing = swing_magnitude(self.t, phase=2.0)
        self.base_pub.publish(
            make_navsatfix(
                stamp,
                "basestation",
                lat=base_swing * math.sin(self.t * 0.25),
                lon=base_swing * math.cos(self.t * 0.15),
                status=NavSatStatus.STATUS_FIX,
            )
        )

        drone_status = NavSatStatus.STATUS_FIX if square_wave(self.t, 14.0, 7.0) else NavSatStatus.STATUS_NO_FIX
        self.drone_pub.publish(
            make_navsatfix(
                stamp,
                "drone",
                lat=42.295 + 0.001 * math.cos(self.t * 0.1),
                lon=-83.712 + 0.001 * math.sin(self.t * 0.1),
                status=drone_status,
            )
        )

    def publish_tf(self, stamp):
        qx, qy, qz, qw = euler_to_quaternion(
            yaw=self.t * 0.1,
            pitch=0.15 * math.sin(self.t * 0.3),
            roll=0.1 * math.cos(self.t * 0.2),
        )

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 10.0 * math.sin(self.t * 0.05)
        t.transform.translation.y = 10.0 * math.cos(self.t * 0.05)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def publish_calibration(self, stamp):
        msg = CalibrationStatus()
        msg.header.stamp = stamp
        msg.magnetometer_calibration = 3 if square_wave(self.t, 6.0, 4.0) else 0
        msg.gyroscope_calibration = 3 if square_wave(self.t, 8.0, 5.0) else 0
        msg.acceleration_calibration = 3 if square_wave(self.t, 10.0, 7.0) else 0
        self.cal_pub.publish(msg)

    def publish_velocity(self, stamp):
        msg = Twist()
        msg.linear.x = 2.0 * math.sin(self.t * 0.5)
        msg.angular.z = 0.8 * math.sin(self.t * 0.3)
        self.vel_pub.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(description="Mock odometry for OdometryReading.vue")
    parser.add_argument("-r", "--rate", type=float, default=5.0, help="Publish rate in Hz (default: 5)")
    parser.add_argument(
        "--blackout-period", type=float, default=60.0, help="Blackout cycle in seconds, 0 to disable (default: 60)"
    )
    parser.add_argument("--blackout-duration", type=float, default=5.0, help="Blackout length in seconds (default: 5)")
    parsed = parser.parse_args()

    rclpy.init(args=args)
    try:
        rclpy.spin(MockOdom(parsed.rate, parsed.blackout_period, parsed.blackout_duration))
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
