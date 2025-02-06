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
<<<<<<< HEAD
        executable="zed",
        name="zed_wrapper",
        parameters=[Path(get_package_share_directory("mrover"), "config", "zed.yaml")],
    )

    object_detector_node = Node(
        package="mrover",
        executable="object_detector",
        name="object_detector",
        parameters=[Path(get_package_share_directory("mrover"), "config", "object_detector.yaml")],
    )

    world_to_zed = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '0', 'map', 'zed_left_camera_frame']
    )

    return LaunchDescription([zed_node, world_to_zed])
=======
        executable="stereo_tag_detector",
        name="stereo_tag_detector",
        parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
    )

    image_tag_detector_node = Node(
        package="mrover",
        executable="image_tag_detector",
        name="image_tag_detector",
        parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
    )

    stereo_object_detector_node = Node(
        package="mrover",
        executable="stereo_object_detector",
        name="stereo_object_detector",
        parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
    )

    image_object_detector_node = Node(
        package="mrover",
        executable="image_object_detector",
        name="image_object_detector",
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

    return launch.LaunchDescription(
        [
            stereo_object_detector_node,
            image_object_detector_node,
            stereo_tag_detector_node,
            image_tag_detector_node,
            zed_node,
            long_range_cam,
        ]
    )
>>>>>>> auton/auton-testing
