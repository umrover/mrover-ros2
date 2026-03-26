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

    return launch.LaunchDescription([keyboard_typing])