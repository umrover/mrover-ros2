#!/usr/bin/env python3


from rclpy.node import Node
import rclpy

from rclpy.executors import ExternalShutdownException
import sys

from mrover.srv import IkTest

import numpy as np



class IK_Testing(Node):

    def __init__(self):
        super().__init__("ik_testing")

    def test_points(self):
        MAX_X = 1.
        MAX_Y = 1.
        MAX_Z = 1.

        pointMap = [[[]]]
        request = IkTest.Request()

        x_idx = 0
        y_idx = 0

        for x in np.arange(-MAX_X, MAX_X, 0.5):
            pointMap.append([[]])
            for y in np.arange(-MAX_Y, MAX_Y, 0.5):
                pointMap[x_idx].append([])
                for z in np.arange(-MAX_Z, MAX_Z, 0.5):
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
                    pointMap[x_idx][y_idx].append (future.result().success)
                y_idx+=1
            x_idx += 1
        
        return pointMap


def main(args = None):
    try:
        rclpy.init()


        ik_tester = IK_Testing()
        pointMap = ik_tester.test_points()
        ik_tester.get_logger().info(f"Point map: {pointMap}")

        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__=="__main__":
    main()
