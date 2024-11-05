#!/usr/bin/env python3


from rclpy.node import Node
import rclpy

from rclpy.executors import ExternalShutdownException
import sys

from mrover.srv import IkTest

import numpy as np

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

import time


class IK_Testing(Node):

    def __init__(self):
        super().__init__("ik_testing")
        self.publisher_ = self.create_publisher(PointCloud2, 'point_cloud', 10)


    def test_points(self, MAX_X = 1., MAX_Y = 1., MAX_Z = 1., STEP = 0.5):

        self.pointRawResults = np.zeros((int(2*MAX_X / STEP) + 1, int(2*MAX_Y / STEP) + 1, int(2*MAX_Z / STEP) + 1), bool)

        # Create a 1D array from -1 to 1 with a step of 0.5
        x_values = np.arange(-MAX_X, MAX_X, STEP)
        y_values = np.arange(-MAX_Y, MAX_Y, STEP)
        z_values = np.arange(-MAX_Z, MAX_Z, STEP)

        X, Y, Z = np.meshgrid(x_values, y_values, z_values)

        self.points = np.vstack([X.ravel(), Y.ravel(), Z.ravel()]).T


        self.point_cloud_data = []

        request = IkTest.Request()

        for x_idx, x in enumerate(x_values):
            for y_idx, y in enumerate(y_values):
                for z_idx, z in enumerate(z_values):
                    #request._target._header._stamp = self.get_clock().now()
                    request.target.header._frame_id = "arm_base_link"
                    request.target.pose._position.x = x
                    request.target.pose._position.y = y
                    request.target.pose._position.z = z
                    client = self.create_client(IkTest, "ik_test")
                    if not client.wait_for_service(timeout_sec=2.0):
                        raise RuntimeError("IK testing not working :(")
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(self, future)
                    success = future.result().success  

                    #Store result in numpy array to print to terminal      
                    self.pointRawResults[x_idx][y_idx][z_idx] = (success)

                    #Store result as point in pointcloud
                    self.point_cloud_data.append([x, y, z, 0, 255, 0]) if success else self.point_cloud_data.append([x, y, z, 255, 0, 0])
        
        self.point_cloud_data = np.array(self.point_cloud_data, dtype=np.float32)





    
    def publishPointCloud(self):
        header = Header(**{"frame_id": "arm_base_link"})
       # Create PointCloud2 message
        pc_data = pc2.create_cloud_xyz32(header, self.points)

        pc_data.fields.append(pc2.PointField(name='x', offset=0, datatype=7, count=1))  # float32
        pc_data.fields.append(pc2.PointField(name='y', offset=4, datatype=7, count=1))  # float32
        pc_data.fields.append(pc2.PointField(name='z', offset=8, datatype=7, count=1))  # float32
        pc_data.fields.append(pc2.PointField(name='r', offset=12, datatype=2, count=1))  # uint8
        pc_data.fields.append(pc2.PointField(name='g', offset=16, datatype=2, count=1))  # uint8
        pc_data.fields.append(pc2.PointField(name='b', offset=20, datatype=2, count=1))  # uint8


        # Create the PointField for RGB
        pc_data.data = self.point_cloud_data.tobytes()  # Convert to bytes for the PointCloud2
        pc_data.is_bigendian = False
        pc_data.point_step = 24  # 3 floats (x, y, z) + 3 bytes (r, g, b)
        pc_data.row_step = pc_data.point_step * len(self.points)
        pc_data.height = 1
        pc_data.width = len(self.points)
        pc_data.is_dense = True


        # Publish the PointCloud2 message
        while (True):
            self.publisher_.publish(pc_data)
            time.sleep(1)
        self.get_logger().info('Publishing colored point cloud data')
        

def main(args = None):
    try:
        rclpy.init()
        np.set_printoptions(threshold=np.inf)


        ik_tester = IK_Testing()
        ik_tester.test_points()
        ik_tester.get_logger().info(f"Point map: {ik_tester.pointRawResults}")
        ik_tester.publishPointCloud()

        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__=="__main__":
    main()
