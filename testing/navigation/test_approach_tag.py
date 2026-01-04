from mrover.msg import GPSWaypoint, WaypointType
from testing.test_infra import MRoverTesting
from pathlib import Path

from launch import LaunchDescription

from mrover.srv import EnableAuton

import rclpy

def enable_auton(node, waypoints: list[GPSWaypoint]):
    node.get_logger().info(__name__)
    node.get_logger().info(f"Received {waypoints}")
    
    enable_auton_service = node.create_client(EnableAuton, "enable_auton")

    while not enable_auton_service.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    req = EnableAuton.Request()
    req.enable = True
    req.waypoints = waypoints
    
    future = enable_auton_service.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    return True

def action2(node):
    node.get_logger().info("no")
    return True

def generate_launch_description():
    MRoverTesting.init()

    MRoverTesting.add_event(enable_auton, {"waypoints": [
        GPSWaypoint(
                tag_id = 0,
                enable_costmap = True,
                latitude_degrees = 42.29321060337751,
                longitude_degrees = -83.70957248114468,
                type = WaypointType(val=WaypointType.POST)
            )
        ]}, 0, Path(__file__))

    MRoverTesting.add_node("nav.py", "navigation", ["navigation.yaml"])

    MRoverTesting.add_node("cost_map", "cost_map", ["perception.yaml"])

    MRoverTesting.add_node("stereo_tag_detector", "stereo_tag_detector", [])

    MRoverTesting.enable_sim()

    return LaunchDescription(MRoverTesting.get_launch_actions())
