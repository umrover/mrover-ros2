import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():
    
    zed_node = Node(
        package="mrover",
        executable="zed",
        name="zed_wrapper",
        parameters=[
            os.path.join(
                get_package_share_directory("mrover"),"config","zed.yaml")
        ]
    )

    zed_node = Node(
        package="mrover",
        executable="object_detector",
        name="object_detector",
        parameters=[
            os.path.join(
                get_package_share_directory("mrover"),"config","object_detector.yaml")
        ]
    )

    return LaunchDescription([
        zed_node
    ])
