import os
from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    mob_streamer_node = Node(
        package="mrover",
        executable="gst_websocket_streamer",
        name="mobility_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    boom_streamer_node = Node(
        package="mrover",
        executable="gst_websocket_streamer",
        name="boom_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    science_streamer_1_node = Node(
        package="mrover",
        executable="gst_websocket_streamer",
        name="science_streamer_1",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    science_streamer_2_node = Node(
        package="mrover",
        executable="gst_websocket_streamer",
        name="science_streamer_2",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )
    return LaunchDescription([science_streamer_1_node, science_streamer_2_node])
