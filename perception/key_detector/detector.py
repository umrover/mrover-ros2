#!/usr/bin/env python3

from logging import debug
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
import time


import pickle
import cv2
import sys
import numpy as np
import cupy as cp
from ultralytics import YOLO

from keyrover import *
from keyrover.vision import *
from keyrover.images.texcoord import *
from keyrover.images.key_mask import *
from keyrover.color import NamedPalette
from keyrover.vision.models import CornersRegressionModel
from keyrover.datasets import KeyboardCornersDataset

import torch
import matplotlib.pyplot as plt


SIZE = (256, 256)
dataset = "v4-nodistort"
CORNER_REGRESSION_MODEL = "magic-wave-28.pt"
YOLO_MODEL = f"{MODELS_PATH}/yolo/train4/weights/best.pt"

class KeyDetector(Node):
    def __init__(self):
        super().__init__('key_detector')

        self.get_logger().info("Starting Key Detector Node...")


        self.test_dataset = KeyboardCornersDataset([], size=SIZE, version=dataset)

        self.corner_regression_model = CornersRegressionModel.load(CORNER_REGRESSION_MODEL)
        self.corner_regression_model.to(device)
        self.corner_regression_model.eval()

        self.yolo_segmentation_model = YOLO(YOLO_MODEL)

        output_video_path = 'output_video.mp4'
        fourcc = cv2.VideoWriter.fourcc(*'XVID')
        fps=20
        self.videoWriter = cv2.VideoWriter(output_video_path, fourcc=fourcc, fps=fps, frameSize=(1440, 1920), isColor=True)


        self.subscription = self.create_subscription(
            ROSImage,
            '/long_range_cam/image',
            self.imageCallback,
            1)

        self.debug_img_pub = self.create_publisher(ROSImage, '/key_detector/img', 1)

    def imageCallback(self, msg):
        self.get_logger().info("Image Callback...")
        start = time.process_time()

        # Convert from ROS msg to np array/torch tensor
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        img_cv = cv2.resize(img, (640, 480), interpolation = cv2.INTER_LINEAR)
        img = self.test_dataset.cvt_image(img_cv)

        imageTime = (time.process_time() -start) * 1000
        start = time.process_time()
        with torch.no_grad():
            # TODO yolo & model to use same input image size

            bboxes = self.yolo_segmentation_model.predict(img_cv, conf=0.3, iou=0.3)[0]
            #print(f'bboxes {bboxes}')
            mask = self.corner_regression_model.predict(img)

            torchTime = (time.process_time() -start) * 1000
            start = time.process_time()

        mask = cv2.resize(mask, bboxes.orig_shape[::-1])
        #print(f'mask {mask}')

        out = plot_yolo(bboxes, draw_text=False, plot=False)
        #imshow(out, mask)
        #describe(mask)

        keys = keys_from_yolo(bboxes)
        texcoords = TexcoordImage(mask * 255, keys)

        with open(f"{DATASETS}/texcoords/{dataset}/texcoords_palette.bin", "rb") as file:
            palette: NamedPalette = pickle.load(file)

        palette.colors

        source = np.array(list(texcoords.texcoords.values()))

        n = len(source)
        m = len(palette.colors)

        source_nvidia = cp.asarray(source)
        palette_colors_nvidia = cp.asarray(palette.colors)

        def create_distance_matrix(A, B):
            if len(A.shape) == 1:
                return None
            #A = cp.repeat(A[:, None, :], len(B), axis=1)
            return cp.linalg.norm(A[:, None, :] - B[None, :, :], axis=-1)

        

        # source (n, 2)-vector of points in 2D
        # palette (m, 2)-vector of points in 2D
        # dist_matrix (n, m) matrix of pairwise distance between source & palette
        distance_matrix = create_distance_matrix(source_nvidia, palette_colors_nvidia)

        if(distance_matrix is None):
            '''
            scale=3
            img = bboxes.orig_img.copy()
            width, height, _ = img.shape
            img = cv2.resize(img, (height * scale, width * scale))
            boxes = [LabeledBBox(*box.xywh[0], label).scale(scale) for box, label in zip(bboxes.boxes, keys)]

            img = to_numpy(img)
            debug_img = ROSImage()
            debug_img.height = img.shape[0]
            debug_img.width = img.shape[1]
            debug_img.encoding = 'bgr8'
            debug_img.is_bigendian = sys.byteorder == 'big'
            debug_img.step = 3 * debug_img.width
            debug_img.data = np.reshape(img, (debug_img.width * debug_img.height * 3)).data

            self.debug_img_pub.publish(debug_img)'''
            return

        # ith row is the ith source point
        # jth column is weight between source and jth target point

        distance_cost_matrix = 1e6 / (distance_matrix ** 3)

        # 0 - weight between 1st source & 1st target (W11)
        # 1 - weight between 1st source & 2nd target (W12)
        # m - weight between 1st source & mth target (W1m)
        # ...
        W = distance_cost_matrix.flatten()
        W = cp.tile(W, 2)

        #print(W.shape)

        targets = cp.tile(palette.colors, (n, 1))
        targets = targets.flatten(order="F") * W

        #print(targets.shape)

        ones = cp.ones((n * m, 1))
        zeros = cp.zeros((n * m, 1))

        repeated_source = cp.repeat(source, m, axis=0)

        P1 = cp.hstack([repeated_source, zeros, zeros, ones, zeros])
        P2 = cp.hstack([zeros, zeros, repeated_source, zeros, ones])
        P = cp.vstack([P1, P2]) * W[:, None]

        (a, b, c, d, tx, ty), _, _, _ = cp.linalg.lstsq(P, targets, rcond=-1)
        A = cp.array([[a.get(), b.get(), tx.get()], [c.get(), d.get(), ty.get()], [0, 0, 1]])

        ones = cp.ones((n, 1))
        result = cp.einsum('ij, bj -> bi', A, cp.hstack([source, ones]))
        result = result[:, :2]


        cost_matrix = create_distance_matrix(result, palette_colors_nvidia)

        cost_matrixTime = (time.process_time() -start) * 1000

        from scipy.optimize import linear_sum_assignment
        row_ind, col_ind = linear_sum_assignment(cost_matrix.get())

        keys = np.array(list(palette.colors_to_name.values()))
        keys = keys[col_ind]

        scale = 3
        def plot_prediction():
            ax = texcoords.scatter()
            scatter(palette.colors, ax=ax, color="black")
            scatter(result, ax=ax, color="red")

            for (u, v), cls in palette.colors_to_name.items():
                ax.text(u - 2, v + 5, cls, c="black", fontsize=5)
                
            for (u, v), cls in zip(result, keys):
                ax.text(u - 2, v - 7, cls, c="red", fontsize=5)

        #plot_prediction()

        #img = bboxes.orig_img #.copy()
        width, height, _ = img_cv.shape
        img = cv2.resize(img_cv, (height * scale, width * scale))
        boxes = [LabeledBBox(*box.xywh[0], label).scale(scale) for box, label in zip(bboxes.boxes, keys)]
        #print(boxes)

        plot_bboxes(img, boxes)

        #img = to_numpy(img)
        for box in boxes:
            draw_textbox(img, box)

        #print('pre pub')
        createTime = (time.process_time() - start) * 1000
        start = time.process_time()
        #img= np.reshape(img, (img.shape[1] * img.shape[0] * 3))
        reshapeTime = (time.process_time() - start) * 1000
        start = time.process_time()

        from cv_bridge import CvBridge

        bridge = CvBridge()
        #debug_img = bridge.cv2_to_imgmsg(img, encoding="bgr8")

        #self.videoWriter.write(img)

        cv2.imshow("Camera", img)
        cv2.waitKey(1)
        #debug_img.height = img.shape[0]
        #debug_img.width = img.shape[1]
        #debug_img.encoding = 'bgr8'
        #debug_img.is_bigendian = sys.byteorder == 'big'
        #debug_img.step = 3 * debug_img.width

        convertTime = (time.process_time() - start) * 1000
        start = time.process_time()
        #self.debug_img_pub.publish(debug_img)

        publishTime = (time.process_time() -start) * 1000
        print("Time to grab image: ", imageTime, " ms")
        print("Time to complete torch model: ",torchTime, " ms")
        print("Time to produce cost_matrix: ",cost_matrixTime, " ms")
        print("Time to create: ", createTime, " ms")
        print("Time to reshape: ",reshapeTime, " ms")
        print("Time to convert: ",convertTime, " ms")
        print("Time to publish: ",publishTime, " ms")
        print("Total time: ", imageTime + torchTime + cost_matrixTime + reshapeTime + publishTime + createTime + convertTime, " ms")

        #plt.show()
    def destroy_node(self):
        self.videoWriter.release()
        super().destroy_node()
        
def main(args=None):
    # Init ROS2
    rclpy.init(args=args)

    # Create and spin node
    key_detector_node = KeyDetector()
    rclpy.spin(key_detector_node)

    # Clean Up
    key_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
