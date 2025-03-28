from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():

    # TODO (ali): add localization.launch

    diff_drive_controller_node = Node(
        package="mrover",
        executable="differential_drive_controller",
        name="differential_drive_controller",
        parameters=[Path(get_package_share_directory("mrover"), "config", "esw.yaml")],
    )

    superstructure_node = Node(
        package="mrover",
        executable="superstructure.py",
        name="superstructure",
        parameters=[Path(get_package_share_directory("mrover"), "config", "superstructure.yaml")],
    )

    # TODO (ali): add LED

    return LaunchDescription([diff_drive_controller_node, superstructure_node])
