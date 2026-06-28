# simulator.launch.py

from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    sim_map_arg = DeclareLaunchArgument("sim_map", default_value="default_map.yaml")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="false")

    share = Path(get_package_share_directory("mrover"))

    simulator_node = Node(
        package="mrover",
        executable="simulator",
        name="simulator",
        output="screen",
        parameters=[
            str(share / "config" / "simulator.yaml"),
            {"sim_map": LaunchConfiguration("sim_map")},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", str(share / "config" / "simulator.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            sim_map_arg,
            rviz_arg,
            simulator_node,
            rviz_node,
        ]
    )
