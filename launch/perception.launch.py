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
        package='mrover',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='mrover',
                plugin='mrover::ObjectDetector',
                name='object_detector',
                parameters=[Path(get_package_share_directory("mrover"), "config", "object_detector.yaml")],
                ),
            ComposableNode(
                package='mrover',
                plugin='mrover::ZedWrapper',
                name='zed_wrapper',
                parameters=[Path(get_package_share_directory("mrover"), "config", "zed.yaml")],
                )
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
    zed_node = Node(
        package="mrover",
        executable="zed",
        name="zed_wrapper",
        
    )

    object_detector_node = Node(
        package="mrover",
        executable="object_detector",
        name="object_detector",
    )

    return LaunchDescription([zed_node, object_detector_node])
