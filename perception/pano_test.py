#!/usr/bin/env python3
import cv2
import os
import glob

image_paths: list[str] = sorted(glob.glob(os.path.join("/home/khush/ros2_ws/src/mrover/data/raw-pano-images/2026-02-14_16:32:06", "*.png")))

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

# label hard coded NESW as a test
fontFace = cv2.FONT_HERSHEY_SIMPLEX # Choose a font type
fontScale = 2.0                 # Font size scale factor
color = (255, 255, 255)           # Text color in BGR format (white in this case)
thickness = 5                   # Line thickness
lineType = cv2.LINE_AA            # For anti-aliased text (smoother edges)

x_org = int(stitched_image.shape[0] / 8)
y_org = int(stitched_image.shape[1] / 4)
x_step = int(2 * stitched_image.shape[0] / 8)
cv2.putText(stitched_image, "HELLO", (x_org,y_org), fontFace, fontScale, color, thickness, lineType)
cv2.imwrite("/home/khush/ros2_ws/src/mrover/data/pano.png", stitched_image)
