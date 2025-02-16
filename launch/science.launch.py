import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


def generate_launch_description():

    launch_include_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mrover"), "launch/base.launch.py")
        )
    )

    launch_include_can = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mrover"), "launch/jetson_can.launch.py")
        )
    )


    diff_drive_controller_node = Node(
        package="mrover",
        executable="differential_drive_controller",
        name="differential_drive_controller",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "esw.yaml")],
    )

    superstructure_node = Node(
        package="mrover",
        executable="superstructure.py",
        name="superstructure",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "superstructure.yaml")],
    )

    science_node = Node(
        package="mrover",
        executable="science_hw_bridge",
        name="science_hw_bridge",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "esw.yaml")],
    )

    return LaunchDescription([science_node, launch_include_can, launch_include_base])