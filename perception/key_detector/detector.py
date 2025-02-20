#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage

import pickle
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

class KeyDetector(Node):
    def __init__(self):
        super().__init__('key_detector')
        dataset = "v4-nodistort"
        test_dataset = KeyboardCornersDataset([], size=SIZE, version=dataset)

        model = CornersRegressionModel.load("magic-wave-28.pt")
        model.to(device)
        model.eval()

        yolo = YOLO(f"{MODELS_PATH}/yolo/train4/weights/best.pt")

        path = f"{DATASETS}/test/image/f52.png"
        img = test_dataset.load_image(path)

        # TODO yolo & model to use same input image size

        bboxes = yolo.predict(path, conf=0.3, iou=0.3)[0]
        mask = model.predict(img)
        mask = cv2.resize(mask, bboxes.orig_shape[::-1])

        out = plot_yolo(bboxes, draw_text=False, plot=False)
        imshow(out, mask)

        describe(mask)

        keys = keys_from_yolo(bboxes)
        texcoords = TexcoordImage(mask * 255, keys)

        with open(f"{DATASETS}/texcoords/{dataset}/texcoords_palette.bin", "rb") as file:
            palette: NamedPalette = pickle.load(file)

        palette.colors

        source = np.array(list(texcoords.texcoords.values()))
        source

        def create_distance_matrix(A, B):
            A = np.repeat(A[:, None, :], len(B), axis=1)
            return np.linalg.norm(A - B, axis=-1)

        n = len(source)
        m = len(palette.colors)

        # source (n, 2)-vector of points in 2D
        # palette (m, 2)-vector of points in 2D
        # dist_matrix (n, m) matrix of pairwise distance between source & palette
        distance_matrix = create_distance_matrix(source, palette.colors)
        distance_matrix

        # ith row is the ith source point
        # jth column is weight between source and jth target point

        distance_cost_matrix = 1e6 / (distance_matrix ** 3)
        distance_cost_matrix

        # 0 - weight between 1st source & 1st target (W11)
        # 1 - weight between 1st source & 2nd target (W12)
        # m - weight between 1st source & mth target (W1m)
        # ...
        W = distance_cost_matrix.flatten()
        W = np.tile(W, 2)

        print(W.shape)
        W

        targets = np.tile(palette.colors, (n, 1))
        targets = targets.flatten(order="F") * W

        print(targets.shape)
        targets

        ones = np.ones((n * m, 1))
        zeros = np.zeros((n * m, 1))

        repeated_source = np.repeat(source, m, axis=0)

        P1 = np.hstack([repeated_source, zeros, zeros, ones, zeros])
        P2 = np.hstack([zeros, zeros, repeated_source, zeros, ones])
        P = np.vstack([P1, P2]) * W[:, None]
        P

        (a, b, c, d, tx, ty), _, _, _ = np.linalg.lstsq(P, targets, rcond=-1)
        A = np.array([[a, b, tx], [c, d, ty], [0, 0, 1]])
        A

        ones = np.ones((n, 1))
        result = np.einsum('ij, bj -> bi', A, np.hstack([source, ones]))
        result = result[:, :2]
        result

        cost_matrix = create_distance_matrix(result, palette.colors)
        cost_matrix

        from scipy.optimize import linear_sum_assignment

        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        row_ind, col_ind

        keys = np.array(list(palette.colors_to_name.values()))
        keys = keys[col_ind]

        ax = texcoords.scatter()
        scatter(palette.colors, ax=ax, color="black")
        scatter(result, ax=ax, color="red")

        for (u, v), cls in palette.colors_to_name.items():
            ax.text(u - 2, v + 5, cls, c="black", fontsize=5)
            
        for (u, v), cls in zip(result, keys):
            ax.text(u - 2, v - 7, cls, c="red", fontsize=5)

        scale = 3

        img = bboxes.orig_img.copy()
        width, height, _ = img.shape
        img = cv2.resize(img, (height * scale, width * scale))
        boxes = [LabeledBBox(*box.xywh[0], label).scale(scale) for box, label in zip(bboxes.boxes, keys)]

        print(boxes)

        plot_bboxes(img, boxes)

        plt.show()

        self.subscription = self.create_subscription(
            ROSImage,
            '/long_range_cam/image',
            self.imageCallback,
            1)
        self.subscription

    def imageCallback(self, msg):
        self.get_logger().info("Image Callback...")
        
        
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
