from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    mob_streamer_node = Node(
        package="mrover",
        executable="gst_camera_server",
        name="mob_streamer",
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

    ee3_streamer_node = Node(
        package="mrover",
        executable="gst_camera_server",
        name="ee3_streamer",
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

    zed_container = ComposableNodeContainer(
        name="zed_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="mrover",
                plugin="mrover::ZedWrapper",
                name="zed_wrapper",
                parameters=[Path(get_package_share_directory("mrover"), "config", "zed.yaml")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="mrover",
                plugin="mrover::GstCameraServer",
                name="zed_streamer",
                parameters=[Path(get_package_share_directory("mrover"), "config", "cameras.yaml")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            mob_streamer_node,
            joint_a_streamer_node,
            ee1_streamer_node,
            ee2_streamer_node,
            ee3_streamer_node,
            zed_mini_streamer_node,
            zed_container,
        ]
    )
