#!/usr/bin/env python3

import sys
import rclpy
import tf2_ros
import numpy as np
import time

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from mrover.srv import IkSample
from rclpy import Parameter
from lie import SE3
from visualization_msgs.msg import Marker
from rclpy.duration import Duration
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header



class IK_Visualization(Node):
    def __init__(self):
        super().__init__("ik_visualizer")

        self.get_logger().info("Starting...")

        # Set up values to obtain arm pos from tf tree
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

        # Set up marker publisher
        self.valid_points_pub = self.create_publisher(Marker, "valid_ik_points", 1)

        # Set up client
        self.client = self.create_client(IkSample, "ik_sample")

        # Ensure Client is Ready
        server_ready = False
        while not server_ready:
            server_ready = self.client.wait_for_service(timeout_sec=1)
            if not server_ready:
                self.get_logger().warn(
                "Service for Arm IKSample is not running", throttle_duration_sec=1
                )

        # Current Data
        self.valid_points = set()
        self.invalid_points = set()
        self.arm_pose = None
        self.step = 0.05 # Temporarily 5cm

        self.get_logger().info("Ready for input...")

        while True:
            time.sleep(0.5)
            if self.visualize():
                self.get_logger().info('Points Visualized')

    def visualize(self) -> bool :
        # Obtain Guarenteed Point
        self.arm_pose = self.get_arm_pose_in_map()

        if self.arm_pose == None:
            self.get_logger().warn("Arm pose is not on the tf tree, unable to visualize")
            return False

        arm_x,arm_y,arm_z = self.arm_pose.translation()

        # Recursivily check points
        self.recursive_check_points(arm_x,arm_y,arm_z)

        # Create Markers for points
        self.publish_valid_points_marker([0,255,0], "valid_ik")

        return True
    
    def recursive_check_points(self, arm_x,arm_y,arm_z):
        request = Vector3(arm_x, arm_y, arm_z)

        if request in self.invalid_points or request in self.valid_points:
            return

        result = self.client.call(request)
        
        if result.valid:
            self.valid_points.add(request)

            # Check +-step in x, y, and z coordinates
            self.recursive_check_points(arm_x - self.step,arm_y, arm_z)
            self.recursive_check_points(arm_x + self.step,arm_y, arm_z)
            self.recursive_check_points(arm_x,arm_y - self.step, arm_z)
            self.recursive_check_points(arm_x,arm_y + self.step, arm_z)
            self.recursive_check_points(arm_x,arm_y, arm_z - self.step)
            self.recursive_check_points(arm_x,arm_y, arm_z + self.step)
        else:
            self.invalid_points.add(request)

        return
    
    def get_arm_pose_in_map(self) -> SE3 | None:
        try:
            return SE3.from_tf_tree(self.tf_buffer, "arm_fk", "base_link")
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            self.get_logger().warn(
                "Failed to get arm pose. Is localization running?", throttle_duration_sec=1
            )
            return None
        
    def publish_valid_points_marker(
        self,
        color: np.ndarray,
        ns: str,
        size=0.2,
        lifetime=0,
    ) -> None:
        if self.get_parameter("display_markers").value:
            points_marker = Marker()
            points_marker.lifetime = Duration(seconds=lifetime).to_msg()
            points_marker.header = Header(frame_id="map", stamp=self.get_clock().now().to_msg())
            points_marker.ns = ns
            points_marker.action = Marker.ADD
            points_marker.color.r = color[0]
            points_marker.color.g = color[1]
            points_marker.color.b = color[2]
            points_marker.color.a = 1.0
            points_marker.pose.orientation.w = 1.0

            for point in self.valid_points:
                assert len(point) > 1, f"Invalid point has size {len(point)}"
                p = Point(x=point.x, y=point.y, z=point.z)
                points_marker.points.append(p)

            points_marker.type = Marker.SPHERE_LIST
            points_marker.id = 0
            points_marker.scale.x = size
            points_marker.scale.y = size

            self.valid_points_pub.publish(points_marker)

    def reset_markers(self) -> None:
        if self.get_parameter("display_markers").value:
            marker = Marker()
            marker.header = Header(frame_id="map", stamp=self.get_clock().now().to_msg())
            marker.action = Marker.DELETEALL
            self.valid_points_pub.publish(marker)


def main():
    try:
        rclpy.init(args=sys.argv)

        ik_node = IK_Visualization()

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    main()
