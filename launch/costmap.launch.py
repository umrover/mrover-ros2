import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    launch_include_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("mrover"), "launch/simulator.launch.py"))
    )

    launch_include_nav = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("mrover"), "launch/navigation.launch.py"))
    )

    costmap_node = Node(
        package="mrover",
        executable="cost_map",
        name="cost_map",
    )

    return LaunchDescription(
        [
            launch_include_sim,
            launch_include_nav,
            costmap_node,
        ]
    )
