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
from sensor_msgs.msg import NavSatFix
from mrover.msg import FixStatus, FixType
from message_filters import TimeSynchronizer, Subscriber


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

        self.gps_sub = Subscriber(self, NavSatFix, "/gps/fix")
        self.gps_status_sub = Subscriber(self, FixStatus, "/gps_fix_status")

        self.synchronizer = TimeSynchronizer([self.gps_sub, self.gps_status_sub], 10)
        self.synchronizer.registerCallback(self.gps_callback)

    def gps_callback(self, gps_msg: NavSatFix, gps_status_msg: FixStatus):
        if (gps_status_msg.fix_type.fix == FixType.NO_SOL):
            self.get_logger().warn("Received invalid GPS data, ignoring")
            return

        x, y, _ = geodetic2enu(gps_msg.latitude, gps_msg.longitude, 0.0, self.ref_lat, self.ref_lon, self.ref_alt, deg=True)
        self.pos_pub.publish(Vector3Stamped(header=gps_msg.header, vector=Vector3(x=x, y=y)))


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
