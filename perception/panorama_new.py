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

# Custom msgs
from mrover.srv import Panorama
from lie import SE3

class Panorama(Node):
    def __init__(self):
        super.__init__('panorama_new')

        # Panorama service, called and then completes pano and returns
        self.pano_srv = self.create_service(Panorama, '/panorama', self.pano_callback)

        # Panorama bools
        self.record_image = False
        self.image_taken = False

        # Subscriber list, to be populated once the service is called and then emptied
        # Subscribers: Image
        self.image_sub = None

        # Debug image publisher
        self.debug_img_pub = self.create_publisher(Image, "/debug_pano", 1)

    def pano_callback(self, _, response):
        self.record_image = True

        # Create subscriber to image 
        self.image_sub = self.create_subscription(Image, "/zed_mini/left/image", self.image_callback, 1)

    def image_callback(self, msg: Image):
        pass
