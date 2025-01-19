import os
from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():

	launch_include_base = IncludeLaunchDescription(
	PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("mrover"), "launch/base.launch.py"))
	)

	launch_include_can = IncludeLaunchDescription(
	PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("mrover"), "launch/jetson_can.launch.py"))
	)

	drive_hw_bridge_node = Node(
    	package="mrover",
    	executable="drive_hw_bridge",
    	name="drive_hw_bridge",
    	parameters=[
    		Path(get_package_share_directory("mrover"), "config", "esw.yaml"),
		],
	)

	return LaunchDescription([launch_include_base, launch_include_can, drive_hw_bridge_node])
