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
        self.cur_gimbal_target = 2*np.pi
        self.num_images = 20
        self.rad_per_image = (2 * np.pi) / self.num_images

        # Heading variables
        self.heading_sub = self.create_subscription(Heading, "/heading/fix", self.heading_callback, 1)
        self.pano_dirs = ['N', 'E', 'S', 'W']
        self.cur_heading = 0.0 # degrees

        # PC Stitching Variables
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
        self.img_dirs = [] # list of imu values per image
        self.headings = []

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
            self.get_logger().info("Updated FOV to {info_msg.fov}")
            self.zed_fov_rad = info_msg.fov * (np.pi / 180)
            self.pixels_per_rad = self.zed_image_width_pixels / self.zed_fov_rad

    def sync_callback(self, pc_msg: PointCloud2, imu_msg: Imu, img: Image, gimbal: ControllerState):
        # extract xyzrgb fields
        # get every tenth point to make the pc sparser
        # TODO: dtype hard-coded to float32
        if self.process_message == False:
            return
        
        if self.zed_fov_rad == None:
            self.get_logger().warn("Wait for FOV to be set")
            return 
        
        # Record the image
        self.current_img = cv2.cvtColor(
            np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, 4), cv2.COLOR_RGBA2RGB
        )

        idx = gimbal.names.index("gimbal_yaw")
        pos = gimbal.positions[idx]

        if self.current_img is None:
            return
        
        self.img_list.append(copy.deepcopy(self.current_img))
        self.img_dirs.append(pos)
        self.headings.append((self.cur_heading + pos) % (2 * np.pi))
        
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

        # Once the image and PC are recorded, move to the next position and 
        # turn off message processing
        self.process_message = False

        # START SPINNING THE MAST GIMBAL TO NEXT POSITION (first position)
        req = ServoPosition.Request()
        req.header = Header()
        req.names = ["gimbal_yaw"]
        req.positions = [self.cur_gimbal_target]
        self.future = self.gimbal_client.call_async(req)
        self.future.add_done_callback(self.gimbal_callback)

    def heading_callback(self, heading: Heading):
        self.get_logger().info("Heading is {heading.heading}")
        self.cur_heading = heading.heading

    def label_pano(self, list_len, pano: np.ndarray, dir_diffs: np.ndarray):
        # label hard coded NESW as a test
        fontFace = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 2.0
        color = (93, 236, 251)
        thickness = 5
        lineType = cv2.LINE_AA
        order = np.arange(0, list_len)

        # For each cardinal dir, find which image has closest heading to that direction
        # Then find the image closest to that one in number from the order list
        # Put the direction where that image "starts" in the pano (assume each indiv.
        # image takes up roughly the same amount of space)
        y_org = int(pano.shape[0] / 4)
        img_mid = pano.shape[1] / (2*len(order))
        for i, dir in enumerate(self.pano_dirs):
            head = i * 90
            diffs = np.abs(self.headings - head)
            closest = diffs.argmin()

            diffs = np.abs(order - int(closest))
            pos = diffs.argmin()

            if abs(self.headings[int(pos)] - self.headings[int(closest)]) > 20:
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

        # START SPINNING THE MAST GIMBAL (first position)
        req = ServoPosition.Request()
        req.header = Header()
        req.names = ["gimbal_yaw"]
        req.positions = [self.cur_gimbal_target]
        self.future = self.gimbal_client.call_async(req)
        self.future.add_done_callback(self.gimbal_callback)

        return response
    
    def gimbal_callback(self, future):
        # Do not process any more images if our target is already 2pi
        if self.cur_gimbal_target == 0.0:
            return 
        
        # set next target and enable img/pc processing callback
        self.cur_gimbal_target -= self.rad_per_image
        self.cur_gimbal_target = max(self.cur_gimbal_target, 0.0)
        self.process_message = True

    def end_callback(self, _, response):
        self.get_logger().info('Ending Pano...')

        self.process_message = False
        self.cur_gimbal_target = 2 * np.pi

        # Return Mast Gimbal to original position
        req = ServoPosition.Request()
        req.header = Header()
        req.names = ["gimbal_yaw"]
        req.positions = [0.0] # TODO is 90 correct?? 0?
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
            name = f"{str(i).zfill(2)}.png"
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
        list_len = len(self.img_list)
        self.img_list = []
        pano = calcPanorama(new_path, shift)

        # Construct Pano and Save, get stitching order
        if pano is not None:
            # label and save the panorama if it succeeds
            self.label_pano(list_len, pano)
            cv2.imwrite(f"{new_path}/pano.png", pano)
            
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
