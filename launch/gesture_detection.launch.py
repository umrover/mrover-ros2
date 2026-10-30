from pathlib import Path

from ament_index_python import get_package_share_directory

import launch
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    gesture_detection_composable_node = ComposableNode(
        package="mrover",
        plugin="mrover::GestureRecognitionNode",
        name="gesture_detection",
        parameters=[Path(get_package_share_directory("mrover"), "config", "gesture_recognition.yaml")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    zed_streamer_annotated_composable_node = ComposableNode(
        package="mrover",
        plugin="mrover::GstCameraServer",
        name="zed_streamer_annotated",
        parameters=[Path(get_package_share_directory("mrover"), "config", "cameras.yaml")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    zed_container = ComposableNodeContainer(
            name="zed_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                ComposableNode(
                    package="mrover",
                    plugin="mrover::ZedWrapper",
                    name="zed_wrapper",
                    parameters=[Path(get_package_share_directory("mrover"), "config", "zed.yaml")],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                ComposableNode(
                    package="mrover",
                    plugin="mrover::GstCameraServer",
                    name="zed_streamer",
                    parameters=[Path(get_package_share_directory("mrover"), "config", "cameras.yaml")],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
            output="screen",
        )

    # create the composed container
    loaded_container = LoadComposableNodes(
        target_container="zed_container",
        composable_node_descriptions=[
            gesture_detection_composable_node,
            zed_streamer_annotated_composable_node,
        ],
    )

    return launch.LaunchDescription([loaded_container, zed_container])
