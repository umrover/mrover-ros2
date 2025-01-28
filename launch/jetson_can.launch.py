from pathlib import Path
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

# Can bridges forward ROS CAN messages to/from the Linux SocketCAN interface
# These correspond one-to-one with the SocketCAN interfaces outputted when running "ip link"

# We use the PEAK PCAN-M.2 card for interfacing with the CAN bus and has four buses
# This driver: https://www.peak-system.com/fileadmin/media/linux/index.htm was compiled on the Jetson
# It makes the SocketCAN interfaces show up in "ip link"


def generate_launch_description():

    can_bridge_1_node = Node(
        package="mrover",
        executable="can_bridge",
        name="can_bridge_1",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "esw.yaml"),
        ],
    )

    can_bridge_2_node = Node(
        package="mrover",
        executable="can_bridge",
        name="can_bridge_2",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "esw.yaml"),
        ],
    )

    can_bridge_3_node = Node(
        package="mrover",
        executable="can_bridge",
        name="can_bridge_3",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "esw.yaml"),
        ],
    )

    can_bridge_4_node = Node(
        package="mrover",
        executable="can_bridge",
        name="can_bridge_4",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "esw.yaml"),
        ],
    )

    return LaunchDescription([can_bridge_3_node])
