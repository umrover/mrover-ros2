from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node


def generate_launch_description():
    cameras_yaml = Path(get_package_share_directory("mrover"), "config", "cameras.yaml")

    mode_arg = DeclareLaunchArgument("mode", default_value="real", description="Launch mode: real or sim")

    camera_client_node = Node(
        package="mrover",
        executable="camera_client",
        name="camera_client",
        parameters=[cameras_yaml],
    )

    # In sim mode, launch gst_camera_server nodes for cameras that use image_topic.
    # Override address to 127.0.0.1 so streams go to localhost.
    long_range_streamer_node = Node(
        package="mrover",
        executable="gst_camera_server",
        name="long_range_streamer",
        output="screen",
        parameters=[cameras_yaml, {"long_range_cam.address": "127.0.0.1"}],
        condition=LaunchConfigurationEquals("mode", "sim"),
    )

    zed_streamer_node = Node(
        package="mrover",
        executable="gst_camera_server",
        name="zed_streamer",
        output="screen",
        parameters=[cameras_yaml, {"zed.address": "127.0.0.1"}],
        condition=LaunchConfigurationEquals("mode", "sim"),
    )

    return LaunchDescription([
        mode_arg,
        camera_client_node,
        long_range_streamer_node,
        zed_streamer_node,
    ])
