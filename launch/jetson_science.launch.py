import os
from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    launch_include_jetson_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            Path(get_package_share_directory("mrover"), "launch", "jetson_base.launch.py").__str__(),
        )
    )

    science_hw_bridge_node = Node(
        package="mrover",
        executable="science_hw_bridge",
        name="science_hw_bridge",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "esw.yaml"),
            Path(get_package_share_directory("mrover"), "config", "science.yaml"),
        ],
    )

    sp1_streamer_node = Node(
        package="mrover",
        executable="gst_camera_server",
        name="sp1_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    sp2_streamer_node = Node(
        package="mrover",
        executable="gst_camera_server",
        name="sp2_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    panorama_node = Node(package="mrover", executable="panorama.py", name="panorama", respawn=True)

    return LaunchDescription([launch_include_jetson_base, science_hw_bridge_node, panorama_node, sp1_streamer_node, sp2_streamer_node])
