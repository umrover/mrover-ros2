# starter_project.launch.py

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():

    navigation_node = Node(
            package="mrover",
            executable="navigation_starter_project.py",
            name="navigation"
    )

    return LaunchDescription([
        # ===========
        # Navigation
        # ===========
        navigation_node
    ])
