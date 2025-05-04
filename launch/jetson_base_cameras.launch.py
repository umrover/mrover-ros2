from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    zed_mini_streamer_node = Node(
        package="mrover",
        executable="gst_camera_server",
        name="zed_mini_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
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

    mob_streamer_node = Node(
        package="mrover",
        executable="gst_camera_server",
        name="mobility_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    static_streamer_node = Node(
        package="mrover",
        executable="gst_camera_server",
        name="static_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    science_streamer_1_node = Node(
        package="mrover",
        executable="gst_camera_server",
        name="science_streamer_1",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    science_streamer_2_node = Node(
        package="mrover",
        executable="gst_camera_server",
        name="science_streamer_2",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    ra_streamer_node = Node(
        package="mrover",
        executable="gst_camera_server",
        name="ra_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
    )

    return LaunchDescription([zed_mini_streamer_node, static_streamer_node])
