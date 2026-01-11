#!/usr/bin/env python3

from mrover.srv import Dummy

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import time


class DummyServer(Node):
    def __init__(self):
        super().__init__('dummy_gimbal')
        self.reentrant_cbg = ReentrantCallbackGroup()

        # Pano Action Server
        self.start_pano = self.create_service(Dummy, 'dummy', self.dummy_callback, callback_group=self.reentrant_cbg)


    def dummy_callback(self, _, response):
        self.get_logger().info("Dummy request recieved, sleeping")
        self.get_clock().sleep_for(Duration(seconds=5))
        self.get_logger().info("Woken up")
        response.success = True
        return response
    

def main(args=None):
    rclpy.init(args=args)
    
    node1 = DummyServer()
    
    # Create the MultiThreadedExecutor, optionally specifying the number of threads
    # If not specified, it uses the number of CPU cores if implemented, otherwise 1
    executor = MultiThreadedExecutor(num_threads=2)
    
    executor.add_node(node1)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node1.get_logger().error(f"An error occurred: {e}")
    finally:
        executor.shutdown()
        node1.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

