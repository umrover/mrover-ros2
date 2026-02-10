from pathlib import Path

from ament_index_python import get_package_share_directory

import launch
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # zed timer composable node
    zed_timer_composable_node = ComposableNode(
        package="mrover",
        plugin="mrover::ZedTimeNode",
        name="zed_timer",
        parameters=[Path(get_package_share_directory("mrover"))],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # create the composed container
    loaded_container = LoadComposableNodes(
        target_container="zed_container",
        composable_node_descriptions=[
            zed_timer_composable_node,
        ],
    )

    return launch.LaunchDescription([loaded_container])
