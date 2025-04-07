import os
from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    light_detector_position_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "zed_left_camera_frame", "map"],
    )

    zed_node = Node(
        package="mrover",
        executable="zed",
        name="zed_wrapper",
        parameters=[Path(get_package_share_directory("mrover"), "config", "zed.yaml")],
    )

    light_detector_node = Node(
        package="mrover",
        executable="light_detector",
        name="light_detector",
        parameters=[Path(get_package_share_directory("mrover"), "config", "light_detector.yaml")],
    )

    return LaunchDescription(
        [
            zed_node,
            light_detector_position_node,
            light_detector_node
        ]
    )