from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ik_visualizer_node = Node(
        package="mrover",
        executable="scripts/ik_visualizer.py",
        name="ik_visualizer",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "navigation.yaml"),
        ],
        respawn=True,
    )

    return LaunchDescription([ik_visualizer_node])
