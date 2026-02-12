import os
from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rover_gps_driver_node = Node(
        package="mrover",
        executable="rover_gps_driver",
        name="rover_gps_driver",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml"), os.path.join(get_package_share_directory("mrover"), "config", "reference_coords.yaml")],
        output="screen",
        respawn=True,
    )

    gps_linearization_node = Node(
        package="mrover",
        executable="gps_linearization.py",
        name="gps_linearization",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml"), os.path.join(get_package_share_directory("mrover"), "config", "reference_coords.yaml")],
        respawn=True,
    )

    heading_filter_node = Node(
        package="mrover",
        executable="heading_filter",
        name="heading_filter",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml"), os.path.join(get_package_share_directory("mrover"), "config", "reference_coords.yaml")],
        respawn=True,
    )

    iekf_se3_node = Node(
        package="mrover",
        executable="iekf_se3",
        name="iekf_se3",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml"), os.path.join(get_package_share_directory("mrover"), "config", "reference_coords.yaml")],
        output="screen",
        respawn=True,
    )

    pose_filter_node = Node(
        package="mrover",
        executable="pose_filter",
        name="pose_filter",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml"), os.path.join(get_package_share_directory("mrover"), "config", "reference_coords.yaml")],
        respawn=True,
    )

    # the 0.55 is because its the right gps
    base_link_to_right_gps = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.55", "0", "0", "0", "0", "1", "gps_frame", "base_link"],
    )

    return LaunchDescription([base_link_to_right_gps, rover_gps_driver_node, gps_linearization_node, heading_filter_node])
