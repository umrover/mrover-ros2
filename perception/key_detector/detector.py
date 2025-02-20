#!/usr/bin/env python3

from logging import debug
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage

import pickle
import cv2
import sys
import numpy as np
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


        self.subscription = self.create_subscription(
            ROSImage,
            '/long_range_cam/image',
            self.imageCallback,
            1)

        self.debug_img_pub = self.create_publisher(ROSImage, '/key_detector/img', 1)

    def imageCallback(self, msg):
        self.get_logger().info("Image Callback...")

        # Convert from ROS msg to np array/torch tensor
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        img_cv = cv2.resize(img, (640, 480), interpolation = cv2.INTER_LINEAR)
        img = self.test_dataset.cvt_image(img_cv)

        # TODO yolo & model to use same input image size

        bboxes = self.yolo_segmentation_model.predict(img_cv, conf=0.3, iou=0.3)[0]
        print(f'bboxes {bboxes}')
        mask = self.corner_regression_model.predict(img)
        mask = cv2.resize(mask, bboxes.orig_shape[::-1])
        print(f'mask {mask}')

        out = plot_yolo(bboxes, draw_text=False, plot=False)
        #imshow(out, mask)

        describe(mask)

        keys = keys_from_yolo(bboxes)
        texcoords = TexcoordImage(mask * 255, keys)

        with open(f"{DATASETS}/texcoords/{dataset}/texcoords_palette.bin", "rb") as file:
            palette: NamedPalette = pickle.load(file)

        palette.colors

        source = np.array(list(texcoords.texcoords.values()))

        def create_distance_matrix(A, B):
            A = np.repeat(A[:, None, :], len(B), axis=1)
            return np.linalg.norm(A - B, axis=-1)

        n = len(source)
        m = len(palette.colors)

        # source (n, 2)-vector of points in 2D
        # palette (m, 2)-vector of points in 2D
        # dist_matrix (n, m) matrix of pairwise distance between source & palette
        distance_matrix = create_distance_matrix(source, palette.colors)

        # ith row is the ith source point
        # jth column is weight between source and jth target point

        distance_cost_matrix = 1e6 / (distance_matrix ** 3)

        # 0 - weight between 1st source & 1st target (W11)
        # 1 - weight between 1st source & 2nd target (W12)
        # m - weight between 1st source & mth target (W1m)
        # ...
        W = distance_cost_matrix.flatten()
        W = np.tile(W, 2)

        print(W.shape)

        targets = np.tile(palette.colors, (n, 1))
        targets = targets.flatten(order="F") * W

        print(targets.shape)

        ones = np.ones((n * m, 1))
        zeros = np.zeros((n * m, 1))

        repeated_source = np.repeat(source, m, axis=0)

        P1 = np.hstack([repeated_source, zeros, zeros, ones, zeros])
        P2 = np.hstack([zeros, zeros, repeated_source, zeros, ones])
        P = np.vstack([P1, P2]) * W[:, None]

        (a, b, c, d, tx, ty), _, _, _ = np.linalg.lstsq(P, targets, rcond=-1)
        A = np.array([[a, b, tx], [c, d, ty], [0, 0, 1]])

        ones = np.ones((n, 1))
        result = np.einsum('ij, bj -> bi', A, np.hstack([source, ones]))
        result = result[:, :2]

        cost_matrix = create_distance_matrix(result, palette.colors)

        from scipy.optimize import linear_sum_assignment

        row_ind, col_ind = linear_sum_assignment(cost_matrix)

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

        img = bboxes.orig_img.copy()
        width, height, _ = img.shape
        img = cv2.resize(img, (height * scale, width * scale))
        boxes = [LabeledBBox(*box.xywh[0], label).scale(scale) for box, label in zip(bboxes.boxes, keys)]

        print(boxes)

        plot_bboxes(img, boxes)

        img = to_numpy(img)
        for box in boxes:
            draw_textbox(img, box)

        print('pre pub')
        debug_img = ROSImage()
        debug_img.height = img.shape[0]
        debug_img.width = img.shape[1]
        debug_img.encoding = 'bgr8'
        debug_img.is_bigendian = sys.byteorder == 'big'
        debug_img.step = 3 * debug_img.width;
        debug_img.data = np.reshape(img, (debug_img.width * debug_img.height * 3)).data

        self.debug_img_pub.publish(debug_img)

        #plt.show()
        
        
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
