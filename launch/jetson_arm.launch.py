from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    launch_include_jetson_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            Path(get_package_share_directory("mrover"), "launch/jetson_base.launch.py").__str__()
        )
    )

    arm_hw_bridge_node = Node(
        package="mrover",
        executable="arm_hw_bridge",
        name="arm_hw_bridge",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "esw.yaml"),
            Path(get_package_share_directory("mrover"), "config", "arm.yaml"),
        ],
    )

    boom_streamer_node = Node(
        package="mrover",
        executable="gst_camera_server",
        name="boom_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    joint_a_streamer_node = Node(
        package="mrover",
        executable="gst_camera_server",
        name="joint_a_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    ee1_streamer_node = Node(
        package="mrover",
        executable="gst_camera_server",
        name="ee1_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    ee2_streamer_node = Node(
        package="mrover",
        executable="gst_camera_server",
        name="ee2_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    zed_mini_streamer_node = Node(
        package="mrover",
        executable="gst_camera_server",
        name="zed_mini_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    launch_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            Path(get_package_share_directory("mrover"), "launch/localization.launch.py").__str__()
        )
    )

    return LaunchDescription(
        [
            launch_include_jetson_base,
            joint_a_streamer_node,
            ee1_streamer_node,
            ee2_streamer_node,
            arm_hw_bridge_node,
        ]
    )
