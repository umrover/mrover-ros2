from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():

    launch_include_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(Path(get_package_share_directory("mrover"), "launch/base.launch.py").__str__())
    )

    launch_include_can = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            Path(get_package_share_directory("mrover"), "launch/jetson_can.launch.py").__str__()
        )
    )

    drive_hw_bridge_node = Node(
        package="mrover",
        executable="drive_hw_bridge",
        name="drive_hw_bridge",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "esw.yaml"),
            Path(get_package_share_directory("mrover"), "config", "drive.yaml"),
        ],
    )

    pdlb_hw_bridge_node = Node(
        package="mrover",
        executable="pdlb_hw_bridge",
        name="pdlb_hw_bridge",
        parameters=[Path(get_package_share_directory("mrover"), "config", "esw.yaml")],
    )

    mast_gimbal_hw_bridge_node = Node(
        package="mrover",
        executable="mast_gimbal_hw_bridge",
        name="mast_gimbal_hw_bridge",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "esw.yaml"),
            Path(get_package_share_directory("mrover"), "config", "mast_gimbal.yaml"),
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
            )
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
            )
        ],
        output="screen",
    )

    return LaunchDescription([
                                                launch_include_base, 
                                               launch_include_can,
                                               drive_hw_bridge_node,
                                               pdlb_hw_bridge_node,
                                               mob_streamer_node,
                                               static_streamer_node,
                                               mast_gimbal_hw_bridge_node,
                                               zed_mini_container,
                                               zed_container])
