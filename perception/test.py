#!/usr/bin/env python3

import numpy as np
import glob

images_dir = "/home/khush/ros2_ws/install/RelWithDebInfo/mrover/share/mrover/../../../../../src/mrover/data/raw-pano-images/2026-03-29_13:19:16/"
files = glob.glob(images_dir + "*.png")
print(images_dir + "*.png")
print(files)