#!/usr/bin/env python3

# rclpy core
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs
import tf2_ros

# python core
import sys
from datetime import datetime
import numpy as np
import cv2

# Custom msgs
from mrover.srv import Panorama
from mrover.srv import ServoPosition
from lie import SE3

class Panorama(Node):
    def __init__(self):
        super.__init__('panorama_new')

        # Panorama service, called and then completes pano and returns
        self.pano_srv = self.create_service(Panorama, '/panorama', self.pano_callback)

        # Subscribe to mast gimbal service
        self.gimbal_srv = self.create_client(ServoPosition, "servo_position")

        # Panorama bools
        self.image_taken = False

        # Subscriber list, to be populated once the service is called and then emptied
        # Subscribers: Image
        self.image_sub = None

        # Debug image publisher
        self.debug_img_pub = self.create_publisher(Image, "/debug_pano", 1)

        # Image stitching variables
        self.stitcher = cv2.Stitcher.create(cv2.Stitcher_PANORAMA)
        self.image_list = []
        self.cam_fov_deg = 105
        self.cam_fov_rad = np.radians(self.cam_fov_deg)
        self.num_rotations = np.ceil(2 * np.pi / self.cam_fov_rad)

        if 2 * np.pi % self.cam_fov_rad == 0:
            self.num_rotations += 1

    def pano_callback(self, _, response):
        self.stitcher = cv2.Stitcher.create(cv2.Stitcher_PANORAMA)

        # Create subscriber to image 
        self.image_sub = self.create_subscription(Image, "/zed_mini/left/image", self.image_callback, 1)

        for i in range(self.num_rotations):
            pos_rad = (2 * np.pi / self.num_rotations) * i
            pass

    def image_callback(self, msg: Image):
        if not self.image_taken:
            self.current_img = cv2.cvtColor(
                np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4), cv2.COLOR_RGBA2RGB
            )

            if self.current_img is not None:
                self.image_list.append(self.current_img)

            self.image_taken = True
