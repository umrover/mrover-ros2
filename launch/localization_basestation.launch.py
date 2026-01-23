from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():

    basestation_gps_driver_node = Node(
        package="mrover",
        executable="basestation_gps_driver.py",
        name="basestation_gps_driver",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "localization.yaml"),
            Path(get_package_share_directory("mrover"), "config", "reference_coords.yaml"),
        ],
    )

    return LaunchDescription(
        [
            basestation_gps_driver_node,
        ]
    )
