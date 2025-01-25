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

    # container = ComposableNodeContainer(
    #     name="perception",
    #     namespace="",
    #     package="rclcpp_components",
    #     executable="component_container_mt",
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package="mrover",
    #             plugin="mrover::StereoObjectDetector",
    #             name="stereo_object_detector_component", 
    #             parameters=[Path(get_package_share_directory("mrover"), "config", "object_detector.yaml")],
    #         ),
    #         ComposableNode(
    #             package="mrover",
    #             plugin="mrover::ImageObjectDetector",
    #             name="image_object_detector_component",
    #             parameters=[Path(get_package_share_directory("mrover"), "config", "object_detector.yaml")],
    #         ),
    #         ComposableNode(
    #             package="mrover",
    #             plugin="mrover::ZedWrapper",
    #             name="zed_component",
    #             parameters=[Path(get_package_share_directory("mrover"), "config", "zed.yaml")],
    #         ),
    #     ],
    # )

    object_detector_node = Node(
        package="mrover",
        executable="object_detector",
        name="object_detector", 
        parameters=[Path(get_package_share_directory("mrover"), "config", "object_detector.yaml")],
    )

    tag_detector_node = Node(
        package="mrover",
        executable="tag_detector",
        name="tag_detector",
        parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
    )

    zed_node = Node(
        package="mrover",
        executable="zed",
        name="zed_wrapper",
        parameters=[Path(get_package_share_directory("mrover"), "config", "zed.yaml")],
    )

    long_range_cam_node = Node(
        package="mrover",
        executable="usb_camera",
        name="long_range_cam"
    )

    zed_link_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0 0 0 0 0 1 base_link zed_left_camera_frame"]
    )

    return launch.LaunchDescription([object_detector_node, tag_detector_node, zed_node, zed_link_tf_node])
