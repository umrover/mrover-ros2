import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    # 1. SETUP PATHS
    pkg_share = get_package_share_directory('mrover')
    aruco_params_file = os.path.join(pkg_share, 'config', 'aruco_params.yaml')
    intrinsics_file = os.path.join(pkg_share, 'config', 'finger_cam_intrinsics.yaml')

    # 2. READ THE INTRINSICS YAML
    with open(intrinsics_file, 'r') as f:
        calib = yaml.safe_load(f)

    # 3. CONVERT TYPES (Strict ROS compliance)
    w = int(calib['image_width'])
    h = int(calib['image_height'])
    k = [float(x) for x in calib['camera_matrix']['data']]
    d = [float(x) for x in calib['distortion_coefficients']['data']]
    p = [float(x) for x in calib['projection_matrix']['data']]
    dist_model = calib['distortion_model']
    frame_id = "finger_camera_link"

    # 4. INLINE CONVERTER SCRIPT (BGRA -> BGR)
    # This reads the raw topic and publishes to '/finger_camera/image_bgr'
    sync_script = f"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageConverter(Node):
    def __init__(self):
        super().__init__('bgra_to_bgr_node')
        # Create Publishers
        self.pub_info = self.create_publisher(CameraInfo, '/finger_camera/camera_info', 10)
        self.pub_img  = self.create_publisher(Image, '/finger_camera/image_bgr', 10)
        
        # Subscribe to RAW topic
        self.sub = self.create_subscription(Image, '/finger_camera/image', self.callback, 10)
        self.bridge = CvBridge()
        
        # Pre-build CameraInfo
        self.info = CameraInfo()
        self.info.width = {w}
        self.info.height = {h}
        self.info.k = {k}
        self.info.d = {d}
        self.info.p = {p}
        self.info.distortion_model = '{dist_model}'
        self.info.header.frame_id = '{frame_id}'

    def callback(self, msg):
        try:
            # Force conversion to BGR (3-channel)
            # We assume input is BGRA or similar
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # If 4 channels (BGRA), convert to BGR
            if len(cv_img.shape) == 3 and cv_img.shape[2] == 4:
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGRA2BGR)
            
            out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            out_msg.header = msg.header
            self.info.header = msg.header
            
            self.pub_img.publish(out_msg)
            self.pub_info.publish(self.info)
        except Exception as e:
            pass # buffer errors

rclpy.init()
node = ImageConverter()
rclpy.spin(node)
"""

    return LaunchDescription([
        # NODE A: The Inline Converter
        ExecuteProcess(
            cmd=['python3', '-c', sync_script],
            name='image_converter',
            output='screen'
        ),

        # NODE B: The ArUco Tracker
        Node(
            package='aruco_opencv',
            executable='aruco_tracker_autostart',
            name='aruco_tracker',
            output='screen',
            parameters=[aruco_params_file],
            remappings=[
                # --- AGGRESSIVE REMAPPING ---
                # We remap every possible alias to our new BGR topic
                ('/finger_camera/image',      '/finger_camera/image_bgr'),
                ('/finger_camera/image_raw',  '/finger_camera/image_bgr'),
                ('image',                     '/finger_camera/image_bgr'),
                ('/camera/image_raw',         '/finger_camera/image_bgr'),
                
                # Also remap camera info just in case
                ('/finger_camera/camera_info', '/finger_camera/camera_info'),
                ('camera_info',                '/finger_camera/camera_info'),
            ]
        )
    ])