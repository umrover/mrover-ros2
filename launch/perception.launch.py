from pathlib import Path

from ament_index_python import get_package_share_directory

import launch
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # image tag detector composable node

    img_tag_detector_composable_node = ComposableNode(
        package="mrover",
        plugin="mrover::ImageTagDetector",
        name="image_tag_detector",
        parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # stereo tag detector composable node
    stereo_tag_detector_composable_node = ComposableNode(
        package="mrover",
        plugin="mrover::StereoTagDetector",
        name="stereo_tag_detector",
        parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # image object detector composable node
    img_object_detector_composable_node = ComposableNode(
        package="mrover",
        plugin="mrover::ImageObjectDetector",
        name="image_object_detector",
        parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # stereo object detector composable node
    stereo_object_detector_composable_node = ComposableNode(
        package="mrover",
        plugin="mrover::StereoObjectDetector",
        name="stereo_object_detector",
        parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # cost map composable node
    cost_map_composable_node = ComposableNode(
        package="mrover",
        plugin="mrover::CostMapNode",
        name="cost_map",
        parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    zed_streamer_annotated_composable_node = ComposableNode(
        package="mrover",
        plugin="mrover::GstCameraServer",
        name="zed_streamer_annotated",
        parameters=[Path(get_package_share_directory("mrover"), "config", "cameras.yaml")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # create the composed container
    loaded_container = LoadComposableNodes(
        target_container="zed_container",
        composable_node_descriptions=[
            stereo_tag_detector_composable_node,
            stereo_object_detector_composable_node,
            cost_map_composable_node,
            zed_streamer_annotated_composable_node,
        ],
    )

    # long range cam node
    long_range_composable_node = ComposableNode(
        package="mrover",
        plugin="mrover::UsbCamera",
        name="long_range_cam",
        parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    long_range_streamer_annotated_composable_node = ComposableNode(
        package="mrover",
        plugin="mrover::GstCameraServer",
        name="long_range_streamer_annotated",
        parameters=[Path(get_package_share_directory("mrover"), "config", "cameras.yaml")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    long_range_container = ComposableNodeContainer(
        name="long_range_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            long_range_composable_node,
            long_range_streamer_annotated_composable_node,
            img_tag_detector_composable_node,
            img_object_detector_composable_node,
        ],
        output="screen",
    )

    return launch.LaunchDescription([loaded_container, long_range_container])
