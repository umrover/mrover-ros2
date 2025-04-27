from pathlib import Path
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    can_bridge_antenna_node = Node(
        package="mrover",
        executable="can_bridge",
        name="can_bridge_antenna",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "antenna.yaml"),
        ],
    )

    antenna_bridge_node = Node(
        package="mrover",
        executable="antenna_bridge",
        name="antenna_bridge",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "antenna.yaml"),
        ],
    )

    return LaunchDescription([can_bridge_antenna_node, antenna_bridge_node])

