#!/usr/bin/env python3

import open3d as o3d

import sys
import os

if len(sys.argv) == 1:
    print("Usage: vis-pc.py <path>")
    exit(1)
else:
    if not os.path.exists(sys.argv[1]):
        print("File does not exist")
        exit(1)

    pcd = o3d.io.read_point_cloud(sys.argv[1])

    o3d.visualization.draw_geometries([pcd])
