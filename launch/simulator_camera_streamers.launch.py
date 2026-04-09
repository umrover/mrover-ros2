import os
from ament_index_python.get_package_share_directory import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = os.path.join(get_package_share_directory('mrover'), 'config', 'sim_cameras.yaml')
    
    zed_streamer = Node(
        package='mrover',
        executable='gst_camera_server',
        name='zed_streamer',
        parameters=[config_path]
    )
    
    long_range_streamer = Node(
        package='mrover',
        executable='gst_camera_server',
        name='long_range_streamer',
        parameters=[config_path]
    )
    
    finger_camera_streamer = Node(
        package='mrover',
        executable='gst_camera_server',
        name='finger_camera_streamer',
        parameters=[config_path]
    )
    
    camera_client = Node(
        package='mrover',
        executable='camera_client',
        name='camera_client',
        parameters=[config_path]
    )
    
    return LaunchDescription([
        zed_streamer,
        long_range_streamer,
        finger_camera_streamer,
        camera_client
    ])
