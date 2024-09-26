#!/usr/bin/env python3
import sys

# python linear algebra library
import numpy as np

# library for interacting with ROS and TF tree
import rclpy
import mrover

from rclpy.action import ActionClient
from rclpy.node import Node

from mrover.action import LanderAlign

from action_msgs.msg import GoalStatus


class ClientDummy(Node):
    def __init__(self):
        super().__init__('client_dummy')
        self._action_client = ActionClient(self, LanderAlign, 'LanderAlignAction')  
        print("finished init")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback')

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result')
        else:
            self.get_logger().info('Goal failed with status')

        # Shutdown after receiving a result
        rclpy.shutdown()

    def send_goal(self):
        print("starting")
        # if not self._action_client.server_is_ready():
        #     print("NOT REEEAAAADDDDYYYYYYYYY")
        #     self._action_client.wait_for_server()

        self._action_client.wait_for_server()

        goal_msg = LanderAlign.Goal()
        print("finished start")


        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        print("f")

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        print("uck")

def main():
    # initialize the node
    
    print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
    rclpy.init(args=sys.argv)
    client_dummy = ClientDummy()
    client_dummy.send_goal()
    rclpy.spin(client_dummy)
    # # # future = ActionClient.cancel_goal() # TODO: THIS IS WRONG FIND THE CORRECT ONE
    # print("guy is here (shit)")
    # ClientDummy().send_goal()
    # rclpy.sleep(4)
    # # # dummy.stop_lander_align()

    # # # let the callback functions run asynchronously in the background
    # print("FUCKING KILL YOURSELF")
    # rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
