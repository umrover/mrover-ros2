from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    teleop_backend = Node(package="mrover", executable="gui_backend.sh", name="teleop_backend")

    teleop_menu = Node(package="mrover", executable="gui_chromium_menu.sh", name="teleop_menu")

    teleop_cameras = Node(package="mrover", executable="gui_chromium_cameras.sh", name="teleop_cameras")

    return LaunchDescription(
        [
            teleop_backend,
            teleop_menu,
            teleop_cameras
        ]
    )
