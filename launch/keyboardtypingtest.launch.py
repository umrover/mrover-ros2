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

    return launch.LaunchDescription([keyboard_typing])