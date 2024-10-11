# starter_project.launch.py

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():

    launch_include_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("mrover"), "launch/simulator.launch.py"))
    )

    perception_node = Node(package="mrover", executable="starter_project_perception", name="perception")

    navigation_node = Node(package="mrover", executable="navigation_starter_project.py", name="navigation")

    localization_node = Node(package="mrover", executable="localization.py", name="localization")

    return LaunchDescription(
        [
            launch_include_sim,
            # ==========
            # Perception
            # ==========
            perception_node,
            # ===========
            # Navigation
            # ===========
            navigation_node,
            # ============
            # Localization
            # ============
            localization_node,
        ]
    )
