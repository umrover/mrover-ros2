#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import matplotlib.pyplot as plt

class PointCloudHistogramNode(Node):
    def __init__(self):
        super().__init__('pointcloud_histogram_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/cost_map/debug_pc',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Convert PointCloud2 msg to numpy array
        points = self.convert_pointcloud2_to_numpy(msg)
      
        # Bin points into width by height buckets
        buckets, ranges = self.bin_points(points)

        # Create histograms based on heights of points in buckets
        histograms, bin_edges = self.create_histograms(buckets)
        
        # Visualize the histograms
        self.visualize_histograms(histograms, bin_edges, ranges)

        # Print or process histograms and ranges further as needed
        self.get_logger().info(f'Histograms: {histograms}')
        self.get_logger().info(f'Ranges: {ranges}')
        
        # Shutdown the node after processing the point cloud
        rclpy.shutdown()

    def convert_pointcloud2_to_numpy(self, cloud_msg):
        # Read the point cloud message into a numpy array
        point_list = []
        for point in point_cloud2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            point_list.append([point[0], point[1], point[2]])
        return np.array(point_list, dtype=np.float32)

    def bin_points(self, points):
        width, height = 10, 10  # Define the number of buckets
        x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
        y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])
        x_edges = np.linspace(x_min, x_max, width + 1)
        y_edges = np.linspace(y_min, y_max, height + 1)
        buckets = [[[] for _ in range(height)] for _ in range(width)]
        ranges = [[(x_edges[i], x_edges[i + 1], y_edges[j], y_edges[j + 1]) for j in range(height)] for i in range(width)]

        for point in points:
            x, y, z = point
            x_idx = np.digitize(x, x_edges) - 1
            y_idx = np.digitize(y, y_edges) - 1
            if 0 <= x_idx < width and 0 <= y_idx < height:
                buckets[x_idx][y_idx].append(z)
        return buckets, ranges

    def create_histograms(self, buckets):
        histograms = []
        bin_edges = None
        for row in buckets:
            row_histograms = []
            for bucket in row:
                if bucket:
                    heights = np.array(bucket)
                    histogram, bin_edges = np.histogram(heights, bins=10)
                else:
                    histogram = np.zeros(10)
                row_histograms.append(histogram)
            histograms.append(row_histograms)
        return histograms, bin_edges

    def visualize_histograms(self, histograms, bin_edges, ranges):
        for i, row in enumerate(histograms):
            num_cols = len(row)
            fig, axs = plt.subplots(nrows=1, ncols=num_cols, figsize=(num_cols * 3, 3))

            for j, histogram in enumerate(row):
                ax = axs[j] if num_cols > 1 else axs
                ax.bar(bin_edges[:-1], histogram, width=np.diff(bin_edges), edgecolor="black", align="edge")
                ax.set_title(f'Bucket ({ranges[i][j][0]:.1f}, {ranges[i][j][1]:.1f}), ({ranges[i][j][2]:.1f}, {ranges[i][j][3]:.1f})', fontsize=8)
                ax.set_xlabel('Height', fontsize=8)
                ax.set_ylabel('Frequency', fontsize=8)
                ax.tick_params(axis='both', which='major', labelsize=6)

            plt.tight_layout(pad=1.0, w_pad=0.5, h_pad=0.5)
            plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudHistogramNode()
    
    # Spin the node so it remains active and waits for the first message
    while rclpy.ok():
        rclpy.spin_once(node)
        # Check if the node has been shutdown during callback execution
        if not rclpy.ok():
            break
            
    # Clean up and shut down properly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
