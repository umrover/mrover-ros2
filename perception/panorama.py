#!/usr/bin/env python3

import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
import tf2_ros
from lie import SE3
from sensor_msgs.msg import Imu
from mrover.srv import CapturePanorama

import rclpy
from rclpy.node import Node
import sys
from datetime import datetime

import cv2
import time
import message_filters

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

class Panorama(Node):
    def __init__(self):
        super().__init__('panorama')

        # Pano Action Server
        self.start_pano = self.create_service(CapturePanorama, '/panorama/start', self.start_callback)
        self.end_pano = self.create_service(CapturePanorama, '/panorama/end', self.end_callback)

        # Start the panorama
        self.record_image = False
        self.record_pc = False

        # PC Stitching Variables
        self.pc_sub = message_filters.Subscriber(self, PointCloud2, "/zed/left/points")
        self.imu_sub = message_filters.Subscriber(self, Imu, "/zed_imu/data_raw")
        self.pc_publisher = self.create_publisher(PointCloud2, "/stitched_pc", 1)

        self.stitched_pc = np.empty((0, 8), dtype=np.float32)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.pc_sub, self.imu_sub], 10, 1)
        self.sync.registerCallback(self.synced_gps_imu_callback)

        # Image Stitching Variables
        self.img_sub = self.create_subscription(Image, "/zed/left/image", self.image_callback, 1);
        self.img_list = []
        self.stitcher = cv2.Stitcher.create()

    def rotate_pc(self, trans_mat: np.ndarray, pc: np.ndarray):
        # rotate the provided point cloud's x, y points by the se3_pose

        # remove NaNs and infs from pc
        pc = pc[np.isfinite(pc).all(axis=1)]

        # represent pc in homogenous coordinates
        points = np.hstack((pc[:, 0:3], np.ones((pc.shape[0], 1))))
        rotated_points = np.matmul(trans_mat, points.T).T
        pc[:, 0:3] = np.delete(rotated_points, 3, 1)
        return pc

    def synced_gps_imu_callback(self, pc_msg: PointCloud2, imu_msg: Imu):
        self.get_logger().info("Point cloud Callback...")
        # extract xyzrgb fields
        # get every tenth point to make the pc sparser
        # TODO: dtype hard-coded to float32
        if self.record_pc:
            self.current_pc = pc_msg
            self.arr_pc = np.frombuffer(bytearray(pc_msg.data), dtype=np.float32).reshape(
                pc_msg.height * pc_msg.width, int(pc_msg.point_step / 4)
            )[0::10, :]

            orientation = np.array([imu_msg.orientation._x, imu_msg.orientation._y, imu_msg.orientation._z, imu_msg.orientation._w])

            orientation = orientation / np.linalg.norm(orientation)

            # Create the SE3
            x, y, z, w = orientation
            rotation = np.array([
                [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w, 0],
                [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w, 0],
                [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2, 0],
                [0, 0, 0, 0]
            ]) 

            rotated_pc = self.rotate_pc(rotation, self.arr_pc)
            self.stitched_pc = np.vstack((self.stitched_pc, rotated_pc))
        else:
            # Clear the stitched pc
            self.stitched_pc = np.empty((0, 8), dtype=np.float32)

    def image_callback(self, msg: Image):
        if self.record_image:
            self.get_logger().info("Image Callback...")
            self.current_img = cv2.cvtColor(
                np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4), cv2.COLOR_RGBA2RGB
            )

            if self.current_img is not None:
                self.img_list.append(np.copy(self.current_img))
        else:
            # Clear the images
            self.img_list = []
            self.stitcher = cv2.Stitcher.create()

    def start_callback(self, _, response):
        self.get_logger().info('Starting Pano...')

        self.record_image = True
        self.record_pc = True

        # START SPINNING THE MAST GIMBAL

        return response

    def end_callback(self, _, response):
        self.get_logger().info('Ending Pano...')

        self.record_image = False
        self.record_pc = False

        # STOP SPINNING THE MAST GIMBAL

        # construct pc from stitched
        try:
            pc_msg = PointCloud2()
            pc_msg.width = self.stitched_pc.shape[0]
            stitched_pc = self.stitched_pc.flatten()
            header = Header()
            header.frame_id = "map"
            pc_msg.header = header
            pc_msg.fields = self.current_pc.fields
            pc_msg.is_bigendian = self.current_pc.is_bigendian
            pc_msg.data = stitched_pc.tobytes()
            pc_msg.height = 1
            pc_msg.point_step = int(len(pc_msg.data) / pc_msg.width)
            pc_msg.is_dense = self.current_pc.is_dense

            self.pc_publisher.publish(pc_msg)
        except:
            # If image succeeds but pc fails, should we set action as succeeded?
            self.get_logger().info("Failed to create point cloud message")

        _, pano = self.stitcher.stitch(self.img_list)

        # Construct Pano and Save
        if pano is not None:
            self.get_logger().info("Saving Pano...")
            now = datetime.now()
            date_string = now.strftime("%H:%M:%S.%f")[:-3];
            cv2.imwrite(f"data/pano-{date_string}.png", pano)
        else:
            self.get_logger().info('Pano Failed...')

        self.get_logger().info('Chill')
        return response

def main(args=None):
    rclpy.init(args=args)

    pano = Panorama()

    rclpy.spin(pano)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pano.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

