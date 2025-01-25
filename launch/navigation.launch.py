import os
from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    navigation_node = Node(
        package="mrover",
        executable="nav.py",
        name="navigation",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "navigation.yaml"),
        ],
    )

    return LaunchDescription([navigation_node])
