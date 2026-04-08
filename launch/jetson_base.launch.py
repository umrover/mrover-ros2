from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    zed_mini_container = ComposableNodeContainer(
        name="zed_mini_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="mrover",
                plugin="mrover::ZedWrapper",
                name="zed_mini_wrapper",
                parameters=[Path(get_package_share_directory("mrover"), "config", "zed_mini.yaml")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="mrover",
                plugin="mrover::GstCameraServer",
                name="zed_mini_streamer",
                parameters=[Path(get_package_share_directory("mrover"), "config", "cameras.yaml")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
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
            zed_mini_container,
            zed_container,
        ]
    )
