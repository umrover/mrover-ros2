import os
from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    launch_include_jetson_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mrover"), "launch/jetson_base.launch.py")
        )
    )

    sa_hw_bridge_node = Node(
        package="mrover",
        executable="sa_hw_bridge",
        name="sa_hw_bridge",
        parameters=[
            Path(get_package_share_directory("mrover"), "config", "esw.yaml"),
            Path(get_package_share_directory("mrover"), "config", "sa.yaml"),
        ],
    )

    panorama_node = Node(package="mrover", executable="panorama.py", name="panorama", respawn=True)

    panorama_rviz = Node(
        package="rviz2",
        namespace="",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", Path(get_package_share_directory("mrover"), "rviz", "panorama.rviz").__str__()],
    )

    zed_mini_container = ComposableNodeContainer(
        name="zed_mini_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="mrover",
                plugin="mrover::ZedWrapper",
                name="zed_mini_wrapper",
                parameters=[Path(get_package_share_directory("mrover"), "config", "zed_mini.yaml")],
                remappings=[
                    ("/zed/right/image", "/zed_mini/right/image"),
                    ("/zed/left/image", "/zed_mini/left/image"),
                    ("/zed_imu/data_raw", "/zed_mini_imu/data_raw"),
                    ("/zed_imu/mag", "/zed_mini_imu/mag"),
                    ("/zed/left/points", "/zed_mini/left/points"),
                    ("/zed/right/camera_info", "/zed_mini/right/camera_info"),
                    ("/zed/left/camera_info", "/zed_mini/left/camera_info"),
                    ("/zed_imu/mag_heading", "/zed_mini_imu/mag_heading"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="mrover",
                plugin="mrover::GstCameraServer",
                name="zed_mini_streamer",
                parameters=[
                    Path(get_package_share_directory("mrover"), "config", "cameras.yaml"),
                    {"zed_mini.image_topic": "/zed_mini/left/image"},
                    {"zed_mini.crop_right": 0},
                    {"zed_mini.stream.width": 1280},
                    {"zed_mini.stream.height": 720},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    launch_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            Path(get_package_share_directory("mrover"), "launch/localization.launch.py").__str__()
        )
    )

    return LaunchDescription(
        [
            launch_include_jetson_base,
            sa_hw_bridge_node,
            panorama_node,
            panorama_rviz,
            zed_mini_container,
            launch_localization
        ]
    )
