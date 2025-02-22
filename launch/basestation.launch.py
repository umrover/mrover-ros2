from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python import get_package_share_directory


def generate_launch_description():

    teleop_backend = Node(package="mrover", executable="gui_backend.sh", name="teleop_backend")

    teleop_frontend = Node(package="mrover", executable="gui_frontend.sh", name="teleop_frontend")

    basestation_gps_driver_node = Node(
        package="mrover",
        executable="basestation_gps_driver.py",
        name="basestation_gps_driver",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml")],
    )


    return LaunchDescription(
        [
            teleop_backend,
            teleop_frontend,
            basestation_gps_driver_node
        ]
    )
