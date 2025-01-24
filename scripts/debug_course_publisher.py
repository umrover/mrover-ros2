#!/usr/bin/env python3

"""
The purpose of this file is for testing courses in the simulator without needing the autonomy GUI.
You can add waypoints with and without tags and these will get published to navigation.
"""

import sys

import numpy as np
import pymap3d

import rclpy
from lie import SE3
from mrover.msg import Waypoint, WaypointType, GPSWaypoint
from mrover.srv import EnableAuton
from rclpy import Parameter
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def publish_waypoints(node: Node, waypoints: list[GPSWaypoint]):
    node.get_logger().info("Waiting for navigation...")
    client = node.create_client(EnableAuton, "enable_auton")
    if not client.wait_for_service(timeout_sec=2.0):
        raise RuntimeError("Enable autonomy service not available. Is navigation running?")

    node.get_logger().info("Navigation available")
    return client.call_async(EnableAuton.Request(enable=True, waypoints=waypoints))


def convert_waypoint_to_gps(reference_point: np.ndarray, waypoint_pose_pair: tuple[Waypoint, SE3]) -> GPSWaypoint:
    waypoint, pose = waypoint_pose_pair
    ref_lat, ref_lon, _ = reference_point
    x, y, z = pose.translation()
    lat, lon, _ = pymap3d.enu2geodetic(x, y, z, ref_lat, ref_lon, 0.0)
    return GPSWaypoint(tag_id=waypoint.tag_id, latitude_degrees=lat, longitude_degrees=lon, type=waypoint.type)


class DebugCoursePublisher(Node):
    def __init__(self):
        super().__init__("debug_course_publisher")

        self.declare_parameters(
            "",
            [
                ("ref_lat", Parameter.Type.DOUBLE),
                ("ref_lon", Parameter.Type.DOUBLE),
                ("ref_alt", Parameter.Type.DOUBLE),
            ],
        )
        ref_point = np.array(
            [
                self.get_parameter("ref_lat").value,
                self.get_parameter("ref_lon").value,
                self.get_parameter("ref_alt").value,
            ]
        )

        future = publish_waypoints(
            self,
            [
                convert_waypoint_to_gps(ref_point, waypoint)
                for waypoint in [
                    (
                        Waypoint(type=WaypointType(val=WaypointType.WATER_BOTTLE)),
                        SE3.from_position_orientation(9.5, 8.0),
                    ),
                    # (
                    #     Waypoint(type=WaypointType(val=WaypointType.NO_SEARCH)),
                    #     SE3.from_position_orientation(20.0, 15.0),
                    # ),
                    # (
                    #     Waypoint(type=WaypointType(val=WaypointType.NO_SEARCH)),
                    #     SE3.from_position_orientation(8.0, -18.0),
                    # ),
                    # (
                    #     Waypoint(type=WaypointType(val=WaypointType.NO_SEARCH)),
                    #     SE3.from_position_orientation(-3.0, 12.0),
                    # ),
                    # (
                    #     Waypoint(type=WaypointType(val=WaypointType.MALLET)),
                    #     SE3.from_position_orientation(-1.0, -1.0),
                    # ),
                    # (
                    #     Waypoint(tag_id=1, type=WaypointType(val=WaypointType.POST)),
                    #     SE3.from_position_orientation(11, -10),
                    # ),
                    # (
                    #     Waypoint(tag_id=1, type=WaypointType(val=WaypointType.MALLET)),
                    #     SE3.from_position_orientation(5, 1),
                    # ),
                ]
            ],
        )

        rclpy.spin_until_future_complete(self, future)


if __name__ == "__main__":
    try:
        rclpy.init(args=sys.argv)
        rclpy.spin(DebugCoursePublisher())
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
