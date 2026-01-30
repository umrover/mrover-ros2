from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_list = []

    def add_detector(cam_name: str):
        cam = Node(
            package="mrover",
            executable="usb_camera",
            name=f"{cam_name}_cam",
            parameters=[
                Path(get_package_share_directory("mrover"), "config", "multi_detection.yaml"),
            ],
            respawn=True,
        )

        tag_detector = Node(
            package="mrover",
            executable="image_tag_detector",
            name=f"{cam_name}_cam_tag_detector",
            parameters=[
                Path(get_package_share_directory("mrover"), "config", "multi_detection.yaml"),
            ],
            respawn=True,
        )

        cam_streamer = Node(
            package="mrover",
            executable="gst_camera_server",
            name=f"{cam_name}_cam_streamer",
            parameters=[
                Path(get_package_share_directory("mrover"), "config", "multi_detection.yaml"),
            ],
            respawn=True,
        )

        launch_list.append(cam)
        launch_list.append(tag_detector)
        launch_list.append(cam_streamer)

    add_detector("laptop")
    add_detector("ardu")

    return LaunchDescription(launch_list)
