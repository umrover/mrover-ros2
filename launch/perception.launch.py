from pathlib import Path

from ament_index_python import get_package_share_directory

import launch
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    loaded_container = LoadComposableNodes(
        target_container="zed_container",
        composable_node_descriptions=[
            ComposableNode(
                package="mrover",
                plugin="mrover::ImageTagDetector",
                name="image_tag_detector",
                parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="mrover",
                plugin="mrover::StereoTagDetector",
                name="stereo_tag_detector",
                parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="mrover",
                plugin="mrover::ImageObjectDetector",
                name="image_object_detector",
                parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="mrover",
                plugin="mrover::StereoObjectDetector",
                name="stereo_object_detector",
                parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="mrover",
                plugin="mrover::CostMapNode",
                name="cost_map",
                parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    usb_cam = Node(
        package="mrover",
        executable="usb_camera",
        name="long_range_cam",
        parameters=[Path(get_package_share_directory("mrover"), "config", "perception.yaml")],
    )

    return launch.LaunchDescription([loaded_container, usb_cam])
