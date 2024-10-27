
from rclpy.node import Node
import rclpy

from rclpy.executors import ExternalShutdownException
import sys

from mrover.srv import IkTest



class IK_Testing(Node):

    def __init__(self, node:Node):
        super().__init__("ik_testing")

    def test_points(self):
        ROWS = 1
        COLS = 1
        pointMap = [[False] * ROWS for i in range(ROWS)]

        row = 0
        col = 0
        request = IkTest.Request()
        request._target._header._stamp = self.node.get_clock().now()
        request.target.header._frame_id = "arm_base_link"
        request.target.pose._position.x = request.target.pose._position.y = request.target.pose._position.z = 0


        client = self.node.create_client(IkTest, "ik_test")
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("IK testing not working :(")

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)        
        pointMap[row][col] = future.result().success
        return pointMap


def main(args = None):
    try:
        rclpy.init(args)


        ik_tester = IK_Testing()
        ik_tester.get_logger().info(ik_tester.test_points())

        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
