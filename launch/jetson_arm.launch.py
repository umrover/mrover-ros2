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

    cameras_config_path = str(Path(get_package_share_directory("mrover"), "config", "cameras.yaml"))

    cameras = [
        "ee1_streamer",
        "ee2_streamer",
        "ee3_streamer",
        "ee4_streamer",
        "ee5_streamer",
    ]

    cam_composable_nodes = [
        ComposableNode(
            package="mrover",
            plugin="mrover::GstCameraServer",
            name=name,
            parameters=[cameras_config_path],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        for name in cameras
    ]

    cam_container = ComposableNodeContainer(
        name="webcam_streamer_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=cam_composable_nodes,
        output="screen",
    )

    # keyboard typing node
    keyboard_typing_node = Node(
        package="mrover",
        executable="keyboard_typing",
        name="keyboard_typing",
        parameters=[Path(get_package_share_directory("mrover"), "config", "keyboard_typing.yaml")],
        respawn=False,
    )

    arm_e_link_to_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.02948991", "0", "-0.0456159", "0", "-1.0996", "0", "arm_joint_DE", "finger_camera_frame"],
    )

    launch_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            Path(get_package_share_directory("mrover"), "launch/localization.launch.py").__str__()
        )
    )

    return LaunchDescription([launch_include_jetson_base, arm_hw_bridge_node, cam_container])
