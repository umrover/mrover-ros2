#!/usr/bin/env python3
import os

# python linear algebra library
import numpy as np

# library for interacting with ROS and TF tree
import rclpy
import mrover

from rclpy.action import ActionClient
from rclpy.node import Node

from mrover.action import LanderAlignAction


class ClientDummy(Node):
    def __init__(self):
        super().__init__('client_dummy')
        self._action_client = ActionClient(self, LanderAlignAction, 'LanderAlignAction')  
        print("finished init")

    def send_goal(self, order):
        print("starting")
        goal_msg = LanderAlignAction.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        print("finished start")
        return self._action_client.send_goal_async(goal_msg)


def main():
    # initialize the node
    print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
    rclpy.init("ClientDummy")
    dummy = ClientDummy()
    future = ActionClient.cancel_goal() # TODO: THIS IS WRONG FIND THE CORRECT ONE

    dummy.start_lander_align()
    rclpy.sleep(4)
    dummy.stop_lander_align()

    # let the callback functions run asynchronously in the background
    rclpy.spin()
    
    
if __name__ == "__main__":
    main()
