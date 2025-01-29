from pathlib import Path

from ament_index_python import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    tag_detector_node = Node(
        package="mrover",
        executable="tag_detector",
        name="tag_detector",
        parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
    )

    zed_node = Node(
        package="mrover",
        executable="zed",
        name="zed",
        parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
    )

    return launch.LaunchDescription([tag_detector_node, zed_node])
