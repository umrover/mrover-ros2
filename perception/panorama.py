#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory

from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from mrover.srv import PanoramaStart, PanoramaEnd, ServoPosition
from mrover.msg import Heading, ControllerState, CameraInfo

import cv2
import time
import message_filters
import os
import numpy as np
import copy
import datetime

import open3d as o3d

from Source import *

class Panorama(Node):
    def __init__(self):
        super().__init__('panorama')

        # Variable for the ZED you'd like to use (zed or zed_mini)
        self.zed_version = "zed_mini"
        self.zed_fov_rad = None
        self.pixels_per_rad = None
        self.zed_image_width_pixels = 1280

        # Pano Action Server
        self.start_pano = self.create_service(PanoramaStart, '/panorama/start', self.start_callback)
        self.end_pano = self.create_service(PanoramaEnd, '/panorama/end', self.end_callback)
        self.gimbal_client = self.create_client(ServoPosition, "gimbal_servo")

        # Start the panorama
        self.process_message = False
        self.fov_sub = self.create_subscription(CameraInfo, f"/{self.zed_version}/left/camera_info", self.fov_callback, 1)

        # gimbal service variables
        self.cur_gimbal_target = 2 * np.pi
        self.num_images = 20
        self.servo_timeout = 3 # seconds
        self.servo_timeout_short = 1 # seconds
        self.start = (300 * np.pi) / 180
        self.end = (60 * np.pi) / 180
        self.delta   = (self.end - self.start) / self.num_images

        # Heading variables
        self.heading_sub = self.create_subscription(Heading, "/heading/fix", self.heading_callback, 1)
        self.pano_dirs = ['E', 'S', 'W', 'N'] # E = 0, N = 270 (as per localization)
        self.cur_heading = 0.0 # degrees

        # PC/Image Stitching Variables
        self.pc_sub = message_filters.Subscriber(self, PointCloud2, f"/{self.zed_version}/left/points")
        self.imu_sub = message_filters.Subscriber(self, Imu, f"/{self.zed_version}_imu/data_raw")
        self.gimbal_sub = message_filters.Subscriber(self, ControllerState, "/gimbal_controller_state")
        self.img_sub = message_filters.Subscriber(self, Image, f"/{self.zed_version}/left/image")
        self.sync = message_filters.ApproximateTimeSynchronizer([self.pc_sub, self.imu_sub, self.img_sub, self.gimbal_sub], 1, 0.5)
        self.sync.registerCallback(self.sync_callback)

        self.pc_publisher = self.create_publisher(PointCloud2, "/stitched_pc", 1)
        self.pano_img_debug_publisher = self.create_publisher(Image, "/debug_pano", 1)
        self.stitched_pc = np.empty((0, 8), dtype=np.float32)

        self.img_list = [] # list of images
        self.img_dirs = [] # gimbal position per image
        self.headings = [] # absolute heading per image

        if not os.path.isdir("data/raw-pano-images"):
            os.mkdir("data/raw-pano-images")


    def rotate_pc(self, trans_mat: np.ndarray, pc: np.ndarray):
        # rotate the provided point cloud's x, y points by the se3_pose

        # remove NaNs and infs from pc
        pc = pc[np.isfinite(pc).all(axis=1)]

        # represent pc in homogenous coordinates
        points = np.hstack((pc[:, 0:3], np.ones((pc.shape[0], 1))))
        rotated_points = np.matmul(trans_mat, points.T).T
        pc[:, 0:3] = np.delete(rotated_points, 3, 1)
        return pc
    
    def fov_callback(self, info_msg: CameraInfo):
        if self.zed_fov_rad is None:
            self.get_logger().info(f"Updated FOV to {info_msg.fov}")
            self.zed_fov_rad = info_msg.fov * (np.pi / 180)
            self.pixels_per_rad = self.zed_image_width_pixels / self.zed_fov_rad

    def sync_callback(self, pc_msg: PointCloud2, imu_msg: Imu, img: Image, gimbal: ControllerState):
        # extract xyzrgb fields
        # get every tenth point to make the pc sparser
        # TODO: dtype hard-coded to float32
        if self.process_message == False or self.zed_fov_rad == None:
            return
        
        target = self.delta * self.pano_position_index + self.start

        if self.start_time is None:
            if self.pano_position_index == self.num_images:
                self.process_message = False
                self.start_time = None
                return
            
            self.start_time = time.monotonic()
            # START SPINNING THE MAST GIMBAL 
            req = ServoPosition.Request()
            req.header = Header()
            req.names = ["gimbal_yaw"]
            req.positions = [target]
            print(f"Sending request to {target}")
            self.gimbal_client.call_async(req)

            return

        if not (time.monotonic() - self.start_time) > (self.servo_timeout if self.pano_position_index == 0 else self.servo_timeout_short):
            return
        
        # Record the image
        self.current_img = cv2.cvtColor(
            np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, 4), cv2.COLOR_RGBA2RGB
        )

        idx = gimbal.names.index("gimbal_yaw")
        pos = gimbal.positions[idx]

        if self.current_img is None:
            return
        
        self.pano_position_index += 1
        self.start_time = None
        
        self.img_list.append(copy.deepcopy(self.current_img))
        self.img_dirs.append(pos)
        self.headings.append((self.cur_heading + pos + np.pi) % (2 * np.pi))
        
        # Record the PC
        self.get_logger().info("Grabbing a PC")
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
            [0, 0, 0, 1]
        ])

        rotated_pc = self.rotate_pc(rotation, self.arr_pc)
        self.stitched_pc = np.vstack((self.stitched_pc, rotated_pc))

    def heading_callback(self, heading: Heading):
        self.cur_heading = heading.heading

    def label_pano(self, num_images, pano: np.ndarray):
        # label hard coded NESW as a test
        fontFace = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 2.0
        color = (93, 236, 251)
        thickness = 5
        lineType = cv2.LINE_AA
        order = np.arange(0, num_images)

        # For each cardinal dir, find which image has closest heading to that direction
        # Then find the image closest to that one in number from the order list
        # Put the direction where that image "starts" in the pano (assume each indiv.
        # image takes up roughly the same amount of space)
        y_org = int(pano.shape[0] / 4)
        img_mid = pano.shape[1] / (2*len(order))
        for i, dir in enumerate(self.pano_dirs):
            head = i * (np.pi / 2)
            diffs = np.abs(np.array(self.headings) - head)
            closest = diffs.argmin()

            diffs = np.abs(order - int(closest))
            pos = diffs.argmin()

            if abs(self.headings[int(pos)] - head) > (20 * np.pi / 180):
                continue
            
            x_org = int(pos * (pano.shape[1] / len(order)) + img_mid)

            cv2.putText(pano, dir, (x_org,y_org), fontFace, fontScale, color, thickness, lineType)
        
        return pano
        
    def start_callback(self, _, response):
        self.get_logger().info('Starting Pano...')

        if self.sync is None: 
            self.pc_sub = message_filters.Subscriber(self, PointCloud2, f"/{self.zed_version}/left/points")
            self.imu_sub = message_filters.Subscriber(self, Imu, f"/{self.zed_version}_imu/data_raw")
            self.gimbal_sub = message_filters.Subscriber(self, ControllerState, "/gimbal_controller_state")
            self.img_sub = message_filters.Subscriber(self, Image, f"/{self.zed_version}/left/image")
            self.sync = message_filters.ApproximateTimeSynchronizer([self.pc_sub, self.imu_sub, self.img_sub, self.gimbal_sub], 1, 0.5)
            self.sync.registerCallback(self.sync_callback)

        if self.heading_sub is not None:
            self.destroy_subscription(self.heading_sub)
            self.heading_sub = None

        self.pano_position_index = 0
        self.start_time = None
        self.process_message = True

        return response

    def end_callback(self, _, response):
        self.get_logger().info('Ending Pano...')

        self.process_message = False
        self.cur_gimbal_target = 2 * np.pi

        # Return Mast Gimbal to original position
        req = ServoPosition.Request()
        req.header = Header()
        req.names = ["gimbal_yaw"]
        req.positions = [np.pi]
        self.gimbal_client.call_async(req)

        # destroy all subs
        if self.sync is not None:
            self.destroy_subscription(self.pc_sub.sub)
            self.destroy_subscription(self.imu_sub.sub)
            self.destroy_subscription(self.img_sub.sub)
            self.destroy_subscription(self.gimbal_sub.sub)
            self.pc_sub = None
            self.imu_sub = None
            self.img_sub = None
            self.gimbal_sub = None
            self.sync = None

        # construct pc from stitched
        self.get_logger().info(f"Shape: {self.stitched_pc.shape}")
        lens = np.linalg.norm(self.stitched_pc[:, 0:3], axis=1)

        filtered_pts = self.stitched_pc[lens < 10, :]

        self.get_logger().info(f"Lens shape: {lens.shape}")

        pcd = o3d.geometry.PointCloud()

        pcd.points = o3d.utility.Vector3dVector(filtered_pts[:, 0:3])

        colors = filtered_pts[:, 3].copy(order='C').view(np.uint8).reshape((filtered_pts.shape[0], 4))[:, 2::-1].copy(order='C').astype(np.float32) / 255

        self.get_logger().info(f"View shape: {colors.shape}")

        pcd.colors = o3d.utility.Vector3dVector(colors)


        # if we do not receive any images, then this will crash
        if len(self.img_list) == 0:
            response.success = False
            self.stitched_pc = np.empty((0, 8), dtype=np.float32)
            self.img_list = []
            self.img_dirs = []
            self.headings = []
            return response

        # Save the images
        unique_id = "{date:%Y-%m-%d_%H:%M:%S}".format(date=datetime.datetime.now())
        share_dir = get_package_share_directory("mrover")
        new_path = f"{share_dir}/../../../../../src/mrover/data/raw-pano-images/{unique_id}/"
        os.mkdir(new_path)
        for i, img in enumerate(self.img_list):
            name = f"{new_path}/{str(i).zfill(2)}.png" # TODO: pathlib
            cv2.imwrite(name, img)

        # stitch the pano together
        self.get_logger().info(f"Stitching {len(self.img_list)} images...")            

        # Calculate shifts
        dirs = np.abs(np.array(self.img_dirs).astype(float))
        diffs = np.abs(np.diff(dirs))
        shift = diffs * self.pixels_per_rad
        shift = shift.astype(int)
        print("Directions (rad): " + str(dirs))
        print("Diffs: " + str(diffs))
        print("Shifts: " + str(shift))

        # clear image list before running calcPano
        num_images = len(self.img_list)
        self.img_list = []
        pano = calcPanorama(new_path, shift)

        # Construct Pano and Save, get stitching order
        if pano is not None:
            # label and save the panorama if it succeeds
            self.label_pano(num_images, pano)
            cv2.imwrite(f"{new_path}/pano.png", pano)
            o3d.io.write_point_cloud(f"{new_path}/pano.ply", pcd)
            
            # convert the panorama to bgra for transport through ROS
            pano = np.clip(pano, 0, 255).astype(np.uint8)
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
        
        # Start capturing heading again
        if self.heading_sub is None:
            self.heading_sub = self.create_subscription(Heading, "/heading/fix", self.heading_callback, 1)
            
        self.stitched_pc = np.empty((0, 8), dtype=np.float32)
        self.img_list = []
        self.img_dirs = []
        self.headings = []
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
