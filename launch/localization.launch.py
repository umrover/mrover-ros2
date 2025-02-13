import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():

    rover_gps_driver_node = Node(
        package="mrover",
        executable="rover_gps_driver",
        name="rover_gps_driver",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml")],
    )

    # rover_gps_driver_node = Node(
    #     package="mrover",
    #     executable="rover_gps_driver.py",
    #     name="rover_gps_driver",
    #     parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml")],
    # )

    gps_linearization_node = Node(
        package="mrover",
        executable="gps_linearization.py",
        name="gps_linearization",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml")],
    )

    zed_node = Node(
        package="mrover",
        executable="zed",
        name="zed_wrapper",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "zed.yaml")],
    )

    heading_filter_node = Node(
        package="mrover",
        executable="heading_filter",
        name="heading_filter",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml")],
    )

    return LaunchDescription([rover_gps_driver_node, gps_linearization_node, zed_node, heading_filter_node])

    # return LaunchDescription([zed_node])
