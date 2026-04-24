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

    joint_a_streamer_node = Node(
        package="mrover",
        executable="gst_camera_server",
        name="joint_a_streamer",
        output="screen",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
        ],
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
        arguments=["0.03713988", "0", "-0.0945642", "0", "-1.134", "0", "arm_fk_de", "finger_camera_frame"],
    )

    launch_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            Path(get_package_share_directory("mrover"), "launch/localization.launch.py").__str__()
        )
    )

    return LaunchDescription(
        [
            launch_include_jetson_base,
            arm_hw_bridge_node,
            ee1_streamer_node,
            ee2_streamer_node,
            ee3_streamer_node,
            joint_a_streamer_node,
            keyboard_typing_node,
            arm_e_link_to_cam,
        ]
    )
