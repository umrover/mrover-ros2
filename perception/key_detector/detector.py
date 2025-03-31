#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
import time

from keyrover import LabeledBBox, plot_bboxes, draw_textbox, ROOT_PATH, plot_yolo
from keyrover.images.key_mask import keys_from_yolo
from keyrover.images.texcoord import TexcoordImage
from keyrover.datasets import KeyboardCornersDataset

import os
from typing import Final
from typing import Literal
from keyrover.color import NamedPalette
from keyrover.vision.models import CornersRegressionModel
import pickle
import cv2
import numpy as np
from ultralytics import YOLO
from torchvision.transforms import v2 as transforms
import torch

DATASETS: Final[str] = f"{ROOT_PATH}/datasets"
MODELS_PATH: Final[str] = f"{ROOT_PATH}/models"
SIZE = (256, 256)
dataset = "v4-nodistort"
CORNER_REGRESSION_MODEL = "magic-wave-28.pt"
YOLO_MODEL = f"{MODELS_PATH}/yolo/train4/weights/best.pt"


DeviceType = Literal["cpu", "cuda", "mps"]
if torch.cuda.is_available():
    device: Final[DeviceType] = "cuda"
elif torch.backends.mps.is_available():
    device: Final[DeviceType] = "mps"
else:
    device: Final[DeviceType] = "cpu"

class KeyDetector(Node):
    def __init__(self):
        super().__init__('key_detector')

        self.get_logger().info("Starting Key Detector Node...")

        self.corner_regression_model = CornersRegressionModel.load(CORNER_REGRESSION_MODEL)
        self.corner_regression_model.to(device)
        self.corner_regression_model.eval()

        self.yolo_segmentation_model = YOLO(YOLO_MODEL)
        self.yolo_segmentation_model.to(device)
        self.yolo_segmentation_model.eval()

        self.subscription = self.create_subscription(
            ROSImage,
            '/long_range_cam/image',
            self.imageCallback,
            1)
    
        self.debug_img_pub = self.create_publisher(ROSImage, '/key_detector/img', 1)

        self.resize = transforms.Resize((256, 256))

        self.test_dataset = KeyboardCornersDataset([], size=SIZE, version=dataset)

    def imageCallback(self, msg):
        self.get_logger().info("Image Callback...")
        start = time.process_time()

        # Convert from ROS msg to np array/torch tensor
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        img_cv = cv2.resize(img, (640, 480), interpolation = cv2.INTER_LINEAR)
        img = self.test_dataset.cvt_image(img_cv)

        imageTime = (time.process_time() - start) * 1000
        start = time.process_time()
        # TODO yolo & model to use same input image size

        bboxes = self.yolo_segmentation_model.predict(img_cv, conf=0.3, iou=0.3)[0]
        mask = self.corner_regression_model.predict(img)

        torchTime = (time.process_time() -start) * 1000
        start = time.process_time()

        mask = cv2.resize(mask, bboxes.orig_shape[::-1])

        keys = keys_from_yolo(bboxes)
        texcoords = TexcoordImage(mask * 255, keys)

        with open(f"{DATASETS}/texcoords/{dataset}/texcoords_palette.bin", "rb") as file:
            palette: NamedPalette = pickle.load(file)

        source = np.array(list(texcoords.texcoords.values()))



        n = len(source)
        m = len(palette.colors)
        def create_distance_matrix(A, B):
            if len(A.shape) == 1:
                return None
            


            A = np.repeat(A[:, None, :], len(B), axis=1)
            print("A's shape: ", A.shape)
            print("B's shape: ", B.shape)

            return np.linalg.norm(A - B, axis=-1)



        # source (n, 2)-vector of points in 2D
        # palette (m, 2)-vector of points in 2D
        # dist_matrix (n, m) matrix of pairwise distance between source & palette
        distance_matrix = create_distance_matrix(source, palette.colors)


        if(distance_matrix is None):
            return
        
        print("distance matrix:",  np.linalg.norm(distance_matrix))

        # ith row is the ith source point
        # jth column is weight between source and jth target point

        distance_cost_matrix = 1e6 / (distance_matrix ** 3)

        # 0 - weight between 1st source & 1st target (W11)
        # 1 - weight between 1st source & 2nd target (W12)
        # m - weight between 1st source & mth target (W1m)
        # ...
        W = distance_cost_matrix.flatten()
        W = np.tile(W, 2)

        targets = np.tile(palette.colors, (n, 1))
        targets = targets.flatten(order="F") * W

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

        cost_matrixTime = (time.process_time() -start) * 1000

        # START HERE
        from scipy.optimize import linear_sum_assignment
        #Hungarian Algorithm
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        keys = np.array(list(palette.colors_to_name.values()))
        keys = keys[col_ind]

        scale = 3

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
