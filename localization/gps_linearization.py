#!/usr/bin/env python3

"""
Converts geodetic GPS coordinates (latitude, longitude, altitude) to local ENU coordinates (x, y, z) using a reference.
These are spherical and cartesian coordinate systems, respectively.
The former would be hard when working on autonomous tasks.
The latter allows us to use the TF tree and simplifies math.

This approximation is valid for small distances around the reference point.
At competition, it should be moved from the Wilson Center to MDRS, or better yet the starting point of the course.
"""

import sys

import numpy as np
from pymap3d.enu import geodetic2enu

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy import Parameter
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
import tf2_ros

import message_filters
from lie.conversions import to_tf_tree, SE3


class GPSLinearization(Node):

    def __init__(self) -> None:
        super().__init__("gps_linearization")

        self.declare_parameters(
            "",
            [
                ("ref_lat", Parameter.Type.DOUBLE),
                ("ref_lon", Parameter.Type.DOUBLE),
                ("ref_alt", Parameter.Type.DOUBLE),
                ("world_frame", Parameter.Type.STRING),
            ],
        )

        self.ref_lat = self.get_parameter("ref_lat").value
        self.ref_lon = self.get_parameter("ref_lon").value
        self.ref_alt = self.get_parameter("ref_alt").value
        self.world_frame = self.get_parameter("world_frame").value

        self.gps_sub = message_filters.Subscriber(self, NavSatFix, "gps/fix")
        self.orientation_sub = message_filters.Subscriber(self, Imu, "zed_imu/data_raw")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.sync = message_filters.ApproximateTimeSynchronizer([self.gps_sub, self.orientation_sub], 10, 1)
        self.sync.registerCallback(self.synced_gps_imu_callback)

    def synced_gps_imu_callback(self, gps_msg: NavSatFix, imu_msg: Imu):

        if np.isnan([gps_msg.latitude, gps_msg.longitude, gps_msg.altitude]).any():
            self.get_logger().warn("Received NaN GPS data, ignoring")
            return

        quaternion = np.array(
            [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w]
        )
        quaternion = quaternion / np.linalg.norm(quaternion)

        x, y, z = geodetic2enu(
            gps_msg.latitude, gps_msg.longitude, gps_msg.altitude, self.ref_lat, self.ref_lon, self.ref_alt, deg=True
        )
        pose = SE3(position=np.array([x, y, 0]), quaternion=quaternion)

        to_tf_tree(tf_broadcaster=self.tf_broadcaster, se3=pose, child_frame="base_link", parent_frame=self.world_frame, stamp=self.get_clock().now().to_msg())

        self.get_logger().info(f"Published to TF Tree: Position({x}, {y}, {z}), Orientation({imu_msg.orientation})")


def main() -> None:
    try:
        rclpy.init(args=sys.argv)
        rclpy.spin(GPSLinearization())
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass


if __name__ == "__main__":
    main()
