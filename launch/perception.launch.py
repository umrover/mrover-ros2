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

    container = ComposableNodeContainer(
        name='perception',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='mrover',
                plugin='mrover::object_detector_component',
                name='object_detector_component',
                parameters=[Path(get_package_share_directory("mrover"), "config", "object_detector.yaml")],
                ),
            ComposableNode(
                package='mrover',
                plugin='mrover::zed_wrapper_component',
                name='zed_wrapper_component',
                parameters=[Path(get_package_share_directory("mrover"), "config", "zed.yaml")],
                )
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
