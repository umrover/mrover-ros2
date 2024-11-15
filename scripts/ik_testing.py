#!/usr/bin/env python3


from numpy._typing._array_like import NDArray
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

import struct


class IK_Testing(Node):

    def __init__(self):
        super().__init__("ik_testing")
        self.publisher_ = self.create_publisher(PointCloud2, 'point_cloud', 10)


    def test_points(self, MAX_X = 1.5, MAX_Y = 0., MAX_Z = 1., STEP = 0.1):


        # Create a 1D array from -1 to 1 with a step of 0.5
        x_values = np.arange(0.1, MAX_X + STEP, STEP)
        y_values = np.array([0.])
        z_values = np.arange(-MAX_Z, MAX_Z + STEP, STEP)

        X, Y, Z = np.meshgrid(x_values, y_values, z_values)

        self.points = np.vstack([X.ravel(), Y.ravel(), Z.ravel()]).T


        self.point_cloud_data = bytes()

        pointRawResults = np.zeros(shape=(len(x_values), len(y_values), len(z_values)), dtype=bool)


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
                    pointRawResults[x_idx][y_idx][z_idx] = (success)

                    #Store result as point in pointcloud
                    self.point_cloud_data += struct.pack("fffBBB", x, y, z, 0, 255, 0) if success else struct.pack("fffBBB", x, y, z, 0, 0, 255)
        self.get_logger().info(f"Point map: {pointRawResults}")

       




    
    def makePointCloud(self):
       # Create PointCloud2 message
        self.pc_data = PointCloud2()
        self.pc_data.header = Header(**{"frame_id": "arm_base_link"})

        self.pc_data.fields.append(pc2.PointField(name='x', offset=0, datatype=7, count=1))  # float32
        self.pc_data.fields.append(pc2.PointField(name='y', offset=4, datatype=7, count=1))  # float32
        self.pc_data.fields.append(pc2.PointField(name='z', offset=8, datatype=7, count=1))  # float32
        self.pc_data.fields.append(pc2.PointField(name='rgb', offset=12, datatype=6, count=1))  # uint32


        # Create the PointField for RGB
        self.pc_data.data = self.point_cloud_data # Convert to bytes for the PointCloud2
        self.pc_data.is_bigendian = False
        self.pc_data.point_step = 15 # 3 floats (x, y, z) + 1 bytes (r, g, b)
        self.pc_data.row_step = self.pc_data.point_step * len(self.points)
        self.pc_data.height = 1
        self.pc_data.width = len(self.points)
        self.pc_data.is_dense = True

        self.get_logger().info('Publishing colored point cloud data')
        self.get_logger().info("Number of points: " + str(len(self.points)))

        self.timer = self.create_timer(1.0, self.publishPointCloud)


    def publishPointCloud(self):
        self.publisher_.publish(self.pc_data)

        

def main(args = None):
    try:
        rclpy.init()
        np.set_printoptions(threshold=np.inf)


        ik_tester = IK_Testing()
        ik_tester.test_points()
        ik_tester.makePointCloud()
        rclpy.spin(ik_tester)

        ik_tester.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__=="__main__":
    main()
