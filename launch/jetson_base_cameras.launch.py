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

    boom_streamer_node = Node(
        package="mrover",
        executable="gst_v4l2_encoder",
        name="boom_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    mob_streamer_node = Node(
        package="mrover",
        executable="gst_v4l2_encoder",
        name="mobility_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    sa_streamer_node = Node(
        package="mrover",
        executable="gst_v4l2_encoder",
        name="sa_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    static_streamer_node = Node(
        package="mrover",
        executable="gst_v4l2_encoder",
        name="static_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    science_streamer_1_node = Node(
        package="mrover",
        executable="gst_v4l2_encoder",
        name="science_streamer_1",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    science_streamer_2_node = Node(
        package="mrover",
        executable="gst_v4l2_encoder",
        name="science_streamer_2",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    ra_streamer_node = Node(
        package="mrover",
        executable="gst_v4l2_encoder",
        name="ra_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    return LaunchDescription([science_streamer_1_node, mob_streamer_node, static_streamer_node, sa_streamer_node])
