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

    zed_component = ComposableNode(
            package="mrover",
            plugin="mrover::ZedWrapper",
            name="zed_component",
            parameters=[Path(get_package_share_directory("mrover"), "config", "zed.yaml")],
            extra_arguments=[{"use_intra_process_comms": True}],
            )
    usb_camera_component = ComposableNode(
            package="mrover",
            plugin="mrover::UsbCamera",
            name="long_range_cam",
            parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
            extra_arguments=[{"use_intra_process_comms": True}],
            )
    key_detector_component = ComposableNode(
            package="mrover",
            plugin="mrover::ImageKeyDetector",
            name="image_key_detector",
            parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
            extra_arguments=[{"use_intra_process_comms": True}],
            )
    image_tag_detector_component = ComposableNode(
            package="mrover",
            plugin="mrover::ImageTagDetector",
            name="image_tag_detector",
            parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
            extra_arguments=[{"use_intra_process_comms": True}],
            )
    stereo_tag_detector_component = ComposableNode(
            package="mrover",
            plugin="mrover::StereoTagDetector",
            name="stereo_tag_detector",
            parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
            extra_arguments=[{"use_intra_process_comms": True}],
            )
    image_object_detector_component = ComposableNode(
            package="mrover",
            plugin="mrover::ImageObjectDetector",
            name="image_object_detector",
            parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
            extra_arguments=[{"use_intra_process_comms": True}],
            )
    stereo_object_detector_component = ComposableNode(
            package="mrover",
            plugin="mrover::StereoObjectDetector",
            name="stereo_object_detector",
            parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
            extra_arguments=[{"use_intra_process_comms": True}],
            )

    container = ComposableNodeContainer(
            name="perception",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                #zed_component,
                usb_camera_component,
                key_detector_component,
                #image_tag_detector_component,
                #stereo_tag_detector_component,
                #image_object_detector_component,
                #stereo_object_detector_component,
                ],
            output="screen",
            )

    return launch.LaunchDescription([container])
