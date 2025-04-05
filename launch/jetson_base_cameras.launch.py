import os
from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    container = ComposableNodeContainer(
        name="cameras",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="mrover",
                plugin="mrover::GstWebsocketStreamer",
                name="mob_streamer",
                parameters=[Path(get_package_share_directory("mrover"), "config", "cameras.yaml")],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
        output="screen",
    )

    # boom_streamer_node = Node(
    #     package="mrover",
    #     executable="gst_websocket_streamer",
    #     name="boom_streamer",
    #     output="screen",
    #     parameters=[
    #         Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
    #     ],
    # )

    # science_streamer_1_node = Node(
    #     package="mrover",
    #     executable="gst_websocket_streamer",
    #     name="science_streamer_1",
    #     output="screen",
    #     parameters=[
    #         Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
    #     ],
    # )

    # science_streamer_2_node = Node(
    #     package="mrover",
    #     executable="gst_websocket_streamer",
    #     name="science_streamer_2",
    #     output="screen",
    #     parameters=[
    #         Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
    #     ],
    # )

    # ra_streamer_node = Node(
    #     package="mrover",
    #     executable="gst_websocket_streamer",
    #     name="ra_streamer",
    #     output="screen",
    #     parameters=[
    #         Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
    #     ],
    # )

    return LaunchDescription([container])
