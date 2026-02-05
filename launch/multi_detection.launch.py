from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node, ComposableNodeContainer

def generate_launch_description():
    launch_list = []

    def add_detector(cam_name: str):
        container = ComposableNodeContainer(
            name=f"{cam_name}_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[],
            output="screen",
        )

        combiner = ComposableNode(
            package="mrover",
            plugin="mrover::Combiner",
            name=f"{cam_name}_cam_detections",
            parameters=[Path(get_package_share_directory("mrover"), "config", "multi_detection.yaml")],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

        if cam_name != "zed":
            cam = ComposableNode(
                package="mrover",
                plugin="mrover::UsbCamera",
                name=f"{cam_name}_cam",
                parameters=[Path(get_package_share_directory("mrover"), "config", "multi_detection.yaml")],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        else:
            cam = ComposableNode(
                package="mrover",
                plugin="mrover::ZedWrapper",
                name=f"zed_wrapper",
                parameters=[
                        Path(get_package_share_directory("mrover"), "config", "multi_detection.yaml"),
                        Path(get_package_share_directory("mrover"), "config", "zed.yaml")
                    ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )


        tag_detector = ComposableNode(
            package="mrover",
            plugin="mrover::ImageTagDetector",
            name=f"{cam_name}_cam_tag_detector",
            parameters=[Path(get_package_share_directory("mrover"), "config", "multi_detection.yaml")],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

        obj_detector = ComposableNode(
            package="mrover",
            plugin="mrover::ImageObjectDetector",
            name=f"{cam_name}_cam_object_detector",
            parameters=[Path(get_package_share_directory("mrover"), "config", "multi_detection.yaml")],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

        cam_streamer = ComposableNode(
            package="mrover",
            plugin="mrover::GstCameraServer",
            name=f"{cam_name}_cam_streamer",
            parameters=[Path(get_package_share_directory("mrover"), "config", "multi_detection.yaml")],
            extra_arguments=[{"use_intra_process_comms": True}],
        )


        load_container = LoadComposableNodes(
            target_container=f"{cam_name}_container",
            composable_node_descriptions=[
                combiner,
                cam,
                tag_detector,
                obj_detector,
                cam_streamer,
            ],
        )

        launch_list.append(container)
        launch_list.append(load_container)

    add_detector("cam1")
    add_detector("cam2")
    add_detector("cam3")
    add_detector("cam4")
    add_detector("zed")

    return LaunchDescription(launch_list)
