import os
from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("mrover")

    launch_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch/simulator.launch.py")),
        launch_arguments={"rviz": "false"}.items(),
    )

    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch/navigation.launch.py"))
    )

    launch_basestation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch/basestation.launch.py")),
        launch_arguments={"mode": LaunchConfiguration("mode")}.items(),
    )

    cost_map_node = Node(
        package="mrover",
        executable="cost_map",
        name="cost_map",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="dev", description="Basestation mode: dev or prod"),
            launch_simulator,
            launch_navigation,
            launch_basestation,
            cost_map_node,
        ]
    )
