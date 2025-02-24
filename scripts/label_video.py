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

sys.path.append('/home/john/ros2_ws/src/mrover/perception/key_detector')

from keyrover import *
from keyrover.vision import *
from keyrover.images.texcoord import *
from keyrover.images.key_mask import *
from keyrover.color import NamedPalette
from keyrover.vision.models import CornersRegressionModel
from keyrover.datasets import KeyboardCornersDataset

import torch
import matplotlib.pyplot as plt

# Define video properties
FRAME_WIDTH = 2160
FRAME_HEIGHT = 3840
FPS = 30
OUTPUT_FILE = 'data/ffmpeg/output.mp4'

SIZE = (256, 256)
dataset = "v4-nodistort"
CORNER_REGRESSION_MODEL = "magic-wave-28.pt"
YOLO_MODEL = f"{MODELS_PATH}/yolo/train4/weights/best.pt"

# Define the codec using VideoWriter_fourcc and create VideoWriter object
FOURCC = cv2.VideoWriter_fourcc(*'mp4v')
WRITER = cv2.VideoWriter(OUTPUT_FILE, FOURCC, FPS, (FRAME_WIDTH, FRAME_HEIGHT))

def read_frames(video_path, output_folder):
    """
    Reads all frames from a video file and saves them as images.

    Args:
        video_path (str): Path to the video file.
        output_folder (str): Path to the folder to save the frames.
    """
    video_capture = cv2.VideoCapture(video_path)
    if not video_capture.isOpened():
        raise Exception(f"Could not open video file: {video_path}")
    
    test_dataset = KeyboardCornersDataset([], size=SIZE, version=dataset)

    corner_regression_model = CornersRegressionModel.load(CORNER_REGRESSION_MODEL)
    corner_regression_model.to(device)
    corner_regression_model.eval()

    yolo_segmentation_model = YOLO(YOLO_MODEL)

    frame_count = 0
    success, image = video_capture.read()
    while success:
        success, image = video_capture.read()
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        try:
            print(f'IMAGE SHAPE: {image.shape}')
            frame_count += 1
            img = test_dataset.cvt_image(image)

            # TODO yolo & model to use same input image size

            bboxes = yolo_segmentation_model.predict(image, conf=0.3, iou=0.3)[0]
            #print(f'bboxes {bboxes}')
            mask = corner_regression_model.predict(img)
            mask = cv2.resize(mask, bboxes.orig_shape[::-1])
            #print(f'mask {mask}')

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

            #print(W.shape)

            targets = np.tile(palette.colors, (n, 1))
            targets = targets.flatten(order="F") * W

            #print(targets.shape)

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

            #print(boxes)

            plot_bboxes(img, boxes)

            img = to_numpy(img)
            for box in boxes:
                draw_textbox(img, box)

            output_path = f"{output_folder}/frame_{frame_count:04d}.jpg"
            cv2.imwrite(output_path, img)

            WRITER.write(img)

            #plt.show()
        except:
            print('failed')

    video_capture.release()
    print(f"Extracted {frame_count} frames.")

if __name__ == "__main__":
    video_path = "/home/john/Downloads/typing_cut.mp4" # Replace with your video path
    output_folder = "/home/john/ros2_ws/src/mrover/data/ffmpeg" # Replace with your desired output folder
    read_frames(video_path, output_folder)

    # Release the VideoWriter object
    WRITER.release()
    print(f"Video saved as {OUTPUT_FILE}")
