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
from tf2_msgs.msg import TFMessage



class IK_Visualization(Node):
    def __init__(self):
        super().__init__("ik_visualizer")
        self.get_logger().info("Starting...")

        # Set up values to obtain arm pos from tf tree
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriber
        self.tf_sub = self.create_subscription(TFMessage, "/tf", self.get_arm_pose_in_map, 1)

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

    def visualize(self) -> bool :
        if self.arm_pose == None:
            return False

        self.get_logger().info('Starting Visualization')

        arm_x,arm_y,arm_z = self.arm_pose.translation()

        # Recursivily check points
        self.recursive_check_points(arm_x,arm_y,arm_z)

        # Create Markers for points
        self.publish_valid_points_marker([0.0,255.0,0.0], "valid_ik")

        self.valid_points = set()
        self.invalid_points = set()
        self.arm_pose = None
        self.get_logger().info('Finished Visualized')
        return True
    
    def recursive_check_points(self, arm_x,arm_y,arm_z):
        request = IkSample.Request()
        request.pos.x = arm_x
        request.pos.y = arm_y
        request.pos.z = arm_z

        if (arm_x,arm_y,arm_z) in self.invalid_points or (arm_x,arm_y,arm_z) in self.valid_points:
            return

        self.get_logger().info("Start Call")

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.2) 
        if future.result() is None:
            return
        
        result = future.result().valid

        self.get_logger().info("Done")
        
        if result:
            self.valid_points.add((arm_x,arm_y,arm_z))

            # Check +-step in x, y, and z coordinates
            self.recursive_check_points(arm_x - self.step,arm_y, arm_z)
            self.recursive_check_points(arm_x + self.step,arm_y, arm_z)
            self.recursive_check_points(arm_x,arm_y - self.step, arm_z)
            self.recursive_check_points(arm_x,arm_y + self.step, arm_z)
            self.recursive_check_points(arm_x,arm_y, arm_z - self.step)
            self.recursive_check_points(arm_x,arm_y, arm_z + self.step)
        else:
            self.invalid_points.add((arm_x,arm_y,arm_z))

        return
    
    def get_arm_pose_in_map(self, msg):
        try:
            self.arm_pose = SE3.from_tf_tree(self.tf_buffer, "arm_fk", "arm_base_link")
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            self.get_logger().warn(
                "Failed to get arm pose. Is localization running?", throttle_duration_sec=1
            )
        
    def publish_valid_points_marker(
        self,
        color: np.ndarray,
        ns: str,
        size=0.2,
        lifetime=0,
    ) -> None:
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
            # assert len(point) > 1, f"Invalid point has size {len(point)}"
            p = Point(x=point[0], y=point[1], z=point[2])
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
        visualized = False

        while True:
            input()
            while (not visualized):
                rclpy.spin_once(ik_node)
                visualized = ik_node.visualize()
            ik_node.get_logger().info("Waiting for input...")
            visualized = False

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    main()
