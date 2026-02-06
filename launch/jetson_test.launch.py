from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    launch_include_can = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            Path(get_package_share_directory("mrover"), "launch/jetson_can.launch.py").__str__()
        )
    )

    motor_test_bridge_node = Node(
        package="mrover",
        executable="motor_test_bridge",
        name="motor_test_bridge",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "esw.yaml"),
            Path(get_package_share_directory("mrover"), "config", "motor.yaml"),
        ],
    )

    return LaunchDescription(
        [
            launch_include_can,
            motor_test_bridge_node,
        ]
    )
