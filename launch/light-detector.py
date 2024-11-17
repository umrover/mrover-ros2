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
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "zed.yaml")],
    )

    base_link_to_zed = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "1", "0", "0", "0", "1", "base_link", "zed_left_camera_frame"],
    )

    light_detector_position_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "mCameraFrame", "mWorldFrame", "100"],
    )

    return LaunchDescription(
        [
            zed_node,
            base_link_to_zed,
            light_detector_position_node,
        ]
    )