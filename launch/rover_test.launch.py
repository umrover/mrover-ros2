import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():

    gps_linearization_node = Node(
        package="mrover",
        executable="gps_linearization.py",
        name="gps_linearization",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml")],
    )

    rover_gps_driver_node = Node(
        package="mrover",
        executable="rover_gps_driver.py",
        name="rover_gps_driver",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml")],
    )

    pose_filter_node = Node(
        package="mrover",
        executable="pose_filter",
        name="pose_filter",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml")],
    )

    zed_node = Node(
        package="mrover",
        executable="zed",
        name="zed_wrapper",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "zed.yaml")],
    )

    base_link_to_zed = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "1", "0", "0", "0", "1", "base_link", "zed_left_camera_frame"],
    )

    map_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "1", "0", "0", "0", "1", "map", "base_link"],
    )

    return LaunchDescription(
        [
            gps_linearization_node,
            #rover_gps_driver_node,
            pose_filter_node,
            zed_node,
            base_link_to_zed,
            map_to_base_link
        ]
    )
