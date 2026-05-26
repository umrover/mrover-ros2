#!/usr/bin/env python3

import open3d as o3d

# Load point cloud from file
pcd = o3d.io.read_point_cloud("output.ply")

# Simple visualization
o3d.visualization.draw_geometries([pcd])
