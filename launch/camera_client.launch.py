from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    camera_client_node = Node(
        package="mrover",
        executable="camera_client",
        name="camera_client",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "multi_detection.yaml"),
        ],
    )

    return LaunchDescription([camera_client_node])
