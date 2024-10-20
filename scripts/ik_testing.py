
from rclpy.node import Node

from mrover.srv import IkTest



class IK_Testing:
    node: Node

    def __init__(self, node:Node):
        self.node = node

    def test_points(self):
        request = IkTest.Request()
        request._target._header._stamp = self.node.get_clock().now()
        request.target.header._frame_id = "test_point"
        request.target.pose._position.x = request.target.pose._position.y = request.target.pose._position.z = 0


        client = self.node.create_client(IkTest, "ik_test")
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("IK testing not working :(")

        future = client.call_async(request)
        future.result.

