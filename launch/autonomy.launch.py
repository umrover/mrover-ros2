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
    # Run Jetson base
    launch_jetson_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(Path(get_package_share_directory("mrover"), "launch/jetson_base.launch.py").__str__())
    )

    # Run Perception
    launch_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(Path(get_package_share_directory("mrover"), "launch/perception.launch.py").__str__())
    )

    # Run Localization
    launch_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(Path(get_package_share_directory("mrover"), "launch/localization.launch.py").__str__())
    )

    # Run Navigation
    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(Path(get_package_share_directory("mrover"), "launch/navigation.launch.py").__str__())
    )

    return LaunchDescription([launch_perception, launch_localization, launch_navigation])
