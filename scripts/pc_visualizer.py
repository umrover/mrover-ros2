import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np

class PointCloudHistogramNode(Node):
    def __init__(self):
        super().__init__('pointcloud_histogram_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/cost_map/debug_pc',
            self.listener_callback,
            1)
        self.subscription

    def listener_callback(self, msg):
        points = self.convert_pointcloud2_to_numpy(msg)
      
        buckets = self.bin_points(points)

        histograms = self.create_histograms(buckets)

        self.get_logger().info(f'Histograms: {histograms}')
        
        rclpy.shutdown()

    def convert_pointcloud2_to_numpy(self, cloud_msg):
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

        for point in points:
            x, y, z = point
            x_idx = np.digitize(x, x_edges) - 1
            y_idx = np.digitize(y, y_edges) - 1
            if 0 <= x_idx < width and 0 <= y_idx < height:
                buckets[x_idx][y_idx].append(z)
        return buckets

    def create_histograms(self, buckets):
        histograms = []
        for row in buckets:
            row_histograms = []
            for bucket in row:
                if bucket:
                    heights = np.array(bucket)
                    histogram, _ = np.histogram(heights, bins=10)
                else:
                    histogram = np.zeros(10)
                row_histograms.append(histogram)
            histograms.append(row_histograms)
        return histograms

def main():
    rclpy.init()
    node = PointCloudHistogramNode()
    
    while rclpy.ok():
        rclpy.spin_once(node)
        if not rclpy.ok():
            break
            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
