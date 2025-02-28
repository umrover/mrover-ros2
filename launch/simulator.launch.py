# simulator.launch.py

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    headless_arg = DeclareLaunchArgument("headless", default_value="false")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")
    object_detector_arg = DeclareLaunchArgument("object_detector", default_value="False")
    cost_map_arg = DeclareLaunchArgument("cost_map", default_value="True")

    launch_include_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("mrover"), "launch/base.launch.py"))
    )

    # TODO (ali): make gst streamer a composed node
    # mrover_container = ComposableNodeContainer(
    #     name="mrover_container",
    #     package="rclcpp_components",
    #     namespace="",
    #     executable="component_container",
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package="mrover",
    #             plugin="mrover::GstWebsocketStreamer",
    #             name="gst_websocket_streamer",
    #             remappings=[("image_topic", "/tag_detection")],
    #             extra_arguments=[{'use_intra_process_comms': True}]
    #         )
    #     ]
    # )

    # gst_websocket_streamer_node = Node(
    #     package="mrover",
    #     executable="gst_websocket_streamer",
    #     name="gst_websocket_streamer",
    #     remappings=[("image_topic", "/tag_detection")]
    # )

    simulator_node = Node(
        package="mrover",
        executable="simulator",
        name="simulator",
        parameters=[
            os.path.join(get_package_share_directory("mrover"), "config", "simulator.yaml"),
            {"headless": LaunchConfiguration("headless")},
        ],
    )

    arm_controller_node = Node(package="mrover", executable="arm_controller", name="arm_controller")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", [os.path.join(get_package_share_directory("mrover"), "config/rviz", "auton_sim.rviz")]],
        condition=LaunchConfigurationEquals("rviz", "true"),
    )

    gps_linearization_node = Node(
        package="mrover",
        executable="gps_linearization.py",
        name="gps_linearization",
        parameters=[os.path.join(get_package_share_directory("mrover"), "config", "localization.yaml")],
    )

    iekf_node = Node(
        package="mrover",
        executable="iekf",
        name="iekf"
    )

    return LaunchDescription(
        [
            headless_arg,
            rviz_arg,
            object_detector_arg,
            cost_map_arg,
            launch_include_base,
            # gst_websocket_streamer_node,
            # launch_include_base,
            simulator_node,
            # arm_controller_node,
            rviz_node,
            gps_linearization_node,
            iekf_node
        ]
    )
