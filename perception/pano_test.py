#!/usr/bin/env python3
import cv2
import os
import glob

image_paths = sorted(glob.glob(os.path.join("/home/khush/ros2_ws/src/mrover/data/raw-pano-images/2026-02-12_11:15:47", "*.png")))

images = []
for path in image_paths:
    img = cv2.imread(path)
    
    if img is not None:
        images.append(img)

stitcher = cv2.Stitcher.create(cv2.Stitcher_PANORAMA)
status, stitched_image = stitcher.stitch(images)

if status == cv2.Stitcher_OK:
    cv2.imwrite("/home/khush/ros2_ws/src/mrover/data/pano.png", stitched_image)
    print(f"Panorama successfully stitched and saved")
else:
    # cv2.imwrite("/home/khush/ros2_ws/src/mrover/data/pano.png", stitched_image)
    print(f"Error during stitching. Status code: {status}")