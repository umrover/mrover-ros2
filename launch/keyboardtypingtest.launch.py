from pathlib import Path

from ament_index_python import get_package_share_directory

import launch
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # keyboard typing node
    keyboard_typing = Node(
        package="mrover",
        executable="keyboard_typing",
        name="keyboard_typing",
        parameters=[Path(get_package_share_directory("mrover"), "config", "keyboard_typing.yaml")],
        respawn=False,
    )

    # gripper: [0.16533, 0, 0.059547] # forward, horizontal, vertical
    # gripper_pitch: 1.134

    # base_link_to_zed = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["0", "0", "1", "0", "0", "0", "1", "base_link", "zed_left_camera_frame"],
    # )

    # joint DE to camera: [0, 0.03713988, 0.0945642] roll: 0, pitch: 1.134, yaw: 0
    arm_e_link_to_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.03713988", "0", "-0.0945642", "0", "-1.134", "0", "arm_fk_c_de", "finger_camera_frame"],
    )

    return launch.LaunchDescription([keyboard_typing, arm_e_link_to_cam])