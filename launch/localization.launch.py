import os
from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # container = ComposableNodeContainer(
    #     name="perception",
    #     namespace="",
    #     package="rclcpp_components",
    #     executable="component_container_mt",
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package="mrover",
    #             plugin="mrover::ZedWrapper",
    #             name="zed_component",
    #             parameters=[Path(get_package_share_directory("mrover"), "config", "zed.yaml")],
    #             extra_arguments=[{"use_intra_process_comms": True}],
    #         ),
    #     ],
    #     output="screen",
    # )

    rover_gps_driver_node = Node(
        package="mrover",
        executable="rover_gps_driver",
        name="rover_gps_driver",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml")],
        output="screen",
        respawn=True
    )

    gps_linearization_node = Node(
        package="mrover",
        executable="gps_linearization.py",
        name="gps_linearization",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml")],
        respawn=True
    )

    heading_filter_node = Node(
        package="mrover",
        executable="heading_filter",
        name="heading_filter",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml")],
        respawn=True
    )

    # iekf_se3_node = Node(
    #     package="mrover",
    #     executable="iekf_se3",
    #     name="iekf_se3",
    #     parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml")],
    #     output="screen",
    #     respawn=True
    # )

    pose_filter_node = Node(
        package="mrover",
        executable="pose_filter",
        name="pose_filter",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml")],
        respawn=True
    )

    return LaunchDescription([rover_gps_driver_node, gps_linearization_node, pose_filter_node])
