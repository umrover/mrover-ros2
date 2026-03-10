#!/usr/bin/env python3

import sys
import math
import rclpy
import tf2_ros
import numpy as np

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from mrover.srv import IkSample
from rclpy import Parameter
from lie import SE3
from visualization_msgs.msg import Marker
from rclpy.duration import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import Header



class IK_Visualization(Node):
    def __init__(self):
        super().__init__("ik_visualizer")

        # Set up values to obtain arm pos from tf tree
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)
        self.world_frame = self.get_parameter("world_frame").value
        self.arm_frame = "arm_base_link"

        # Set up marker publisher
        self.valid_points_pub = self.create_publisher(Marker, "valid_ik_points", 1)

        # Set up client
        self.client = self.create_client(IkSample, "ik_sample")

        # Current Data
        self.current_points = {}
        self.arm_pose = None
        self.step = 0.05 # Temporarily 5cm

    def visualize(self):
        # Obtain Guarenteed Point
        self.arm_pose = self.get_pose_in_map()
        arm_position = self.arm_pose.translation()[:2]

        # Recursivily check points
        self.recursive_check_points(arm_position)

        # Create Markers for points

        return 
    
    def recursive_check_points(self, point):

        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.future.result()

        return
    
    def get_arm_pose_in_map(self) -> SE3 | None:
        try:
            return SE3.from_tf_tree(self.tf_buffer, self.arm_frame, self.world_frame)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            self.get_logger().warn(
                "Failed to get arm pose. Is localization running?", throttle_duration_sec=1
            )
            return None
        
    def publish_point_marker(
        self,
        points: np.ndarray,
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

            for point in points:
                assert len(point) > 1, f"Invalid point has size {len(point)}"
                p = Point(x=point[0], y=point[1])
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
        
        while True:
            input()
            rclpy.spin_once(ik_node)
            print('Updated')
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        ik_node.reset_markers()
        sys.exit(1)


if __name__ == "__main__":
    main()
