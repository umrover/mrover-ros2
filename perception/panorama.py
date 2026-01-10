#!/usr/bin/env python3

import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from mrover.srv import ServoPosition, Pano

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import cv2
import time
import message_filters
import os
import sys
from datetime import datetime

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

# Controls execution rate to be at a given hz
class PanoRate():
    def __init__(self, rate, node):
        self.rate = rate
        self.node = node
        self.curr_time = self.node.get_clock().now()

    def sleep(self):
        self.prev_time = self.curr_time
        self.curr_time = self.node.get_clock().now()

        # If this is the first loop through don't block
        if self.prev_time is None:
            return

        # Spin until rate has been met
        end_timestamp = (self.prev_time.nanoseconds + ((1.0 / self.rate) * 1e9))
        begin_timestamp = self.node.get_clock().now().nanoseconds
        duration_nanoseconds = end_timestamp - begin_timestamp
        duration_seconds = duration_nanoseconds * 1e-9
        if(duration_seconds > 0):
            time.sleep(duration_seconds)

class Panorama(Node):
    def __init__(self):
        super().__init__('panorama')

        # Pano Action Server
        self.start_pano = self.create_service(Pano, 'panorama', self.pano_callback)
        self.gimbal_client = self.create_client(ServoPosition, "gimbal_servo")

        # PC Stitching Variables
        self.pc_sub = message_filters.Subscriber(self, PointCloud2, "/zed_mini/left/points")
        self.imu_sub = message_filters.Subscriber(self, Imu, "/zed_mini_imu/data_raw")
        self.subs = [self.pc_sub, self.imu_sub]
        self.pc_publisher = self.create_publisher(PointCloud2, "/stitched_pc", 1)
        self.pano_img_debug_publisher = self.create_publisher(Image, "/debug_pano", 1)
        self.pc_rate = PanoRate(2, self)

        self.stitched_pc = np.empty((0, 8), dtype=np.float32)

        # Image Stitching Variables
        self.img_sub = None
        self.img_list = []
        self.stitcher = cv2.Stitcher.create()
        self.img_rate = PanoRate(2, self)


    def rotate_pc(self, trans_mat: np.ndarray, pc: np.ndarray):
        # rotate the provided point cloud's x, y points by the se3_pose

        # remove NaNs and infs from pc
        pc = pc[np.isfinite(pc).all(axis=1)]

        # represent pc in homogenous coordinates
        points = np.hstack((pc[:, 0:3], np.ones((pc.shape[0], 1))))
        rotated_points = np.matmul(trans_mat, points.T).T
        pc[:, 0:3] = np.delete(rotated_points, 3, 1)
        return pc

    def synced_gps_pc_callback(self, pc_msg: PointCloud2, imu_msg: Imu):
        # extract xyzrgb fields
        # get every tenth point to make the pc sparser
        # TODO: dtype hard-coded to float32
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
        self.pc_rate.sleep()

    def image_callback(self, msg: Image):
        self.current_img = cv2.cvtColor(
            np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4), cv2.COLOR_RGBA2RGB
        )

        if self.current_img is not None:
            self.img_list.append(np.copy(self.current_img))
        self.img_rate.sleep()

    def pano_callback(self, _, response):
        self.get_logger().info('Starting Pano...')
        self.sync = message_filters.ApproximateTimeSynchronizer([self.pc_sub, self.imu_sub], 10, 1)
        self.sync.registerCallback(self.synced_gps_pc_callback)
        self.img_sub = self.create_subscription(Image, "/zed_mini/left/image", self.image_callback, 1)

        end_time = time.time() + 20

        while time.time() < end_time:
            pass

        self.img_sub = None
        self.sync = None

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
            self.get_logger().info("Failed to create point cloud message...")

        # if we do not receive any images, then this will crash
        if len(self.img_list) == 0:
            response.success = False
            return response

        # Save the images
        unique_id = "{date:%Y-%m-%d_%H:%M:%S}".format(date=datetime.datetime.now())
        new_path = f"data/raw-pano-images/{unique_id}"
        os.mkdir(new_path)
        for i, img in enumerate(self.img_list):
            cv2.imwrite(f"{new_path}/{i}.png", img)              

        # stitch the pano together
        self.get_logger().info(f"Stitching {len(self.img_list)} images...")
        _, pano = self.stitcher.stitch(self.img_list)

        # Construct Pano and Save
        if pano is not None:
            # save the panorama if it succeeds
            cv2.imwrite(f"{new_path}/pano.png", img)
            
            # convert the panorama to bgra for transport through ROS
            bgra_pano = cv2.cvtColor(pano, cv2.COLOR_BGR2BGRA)

            # fill out the image message
            response.success = True
            response.img.header = Header()
            response.img.width = bgra_pano.shape[1]
            response.img.height = bgra_pano.shape[0]
            response.img.encoding = 'bgra8'
            response.img.is_bigendian = 0
            response.img.step = bgra_pano.shape[1] * 4

            # ensure all values are between 0 and 255
            np.clip(bgra_pano, 0, 255)

            # copy the panorama data into the message
            response.img.data = bgra_pano.astype(np.uint8).tobytes()

            # publish for debug viewing
            self.pano_img_debug_publisher.publish(response.img)
        else:
            # pano stitcher failed
            self.get_logger().info('Pano Failed...')
            response.success = False
            return response
        
        # clear pano and image list
        self.stitched_pc = np.empty((0, 8), dtype=np.float32)
        self.img_list = []
        self.stitcher = cv2.Stitcher.create(cv2.Stitcher_PANORAMA)

        self.sync.disconnect()
        self.img_sub = None
        
        self.get_logger().info('Pano response sent to frontend')
        return response

def main(args=None):
    rclpy.init(args=args)

    pano = Panorama()

    rclpy.spin(pano)

    pano.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

