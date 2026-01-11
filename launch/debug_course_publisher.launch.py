from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nav_node = Node(
        package="mrover",
        executable="debug_course_publisher.py",
        name="debug_course_publisher",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "navigation.yaml"),
            Path(get_package_share_directory("mrover"), "config", "reference_coords.yaml"),
        ],
        respawn=True,
    )

    return LaunchDescription([nav_node])