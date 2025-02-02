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
    stereo_tag_detector_node = Node(
        package="mrover",
        executable="stereo_tag_detector",
        name="image_tag_detector",
        parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
    )

    image_tag_detector_node = Node(
        package="mrover",
        executable="image_tag_detector",
        name="image_tag_detector",
        parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
    )

    long_range_cam = Node(
        package="mrover",
        executable="usb_camera",
        name="long_range_cam",
        parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
    )

    zed_node = Node(
        package="mrover",
        executable="zed",
        name="zed",
        parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
    )

    return launch.LaunchDescription([stereo_tag_detector_node, image_tag_detector_node, zed_node, long_range_cam])
