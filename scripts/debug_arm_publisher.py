#!/usr/bin/env python3


import sys

import rclpy
from lie import SE3
from mrover.msg import Position, Velocity, Throttle
from mrover.action import TypingPosition
from rclpy.publisher import Publisher
from std_msgs.msg import Header
from rclpy.node import Node
from rclpy.action import ActionClient
from enum import Enum
from rclpy.executors import ExternalShutdownException

class Mode(Enum):
    THR = 0
    POS = 1
    VEL = 2
    IK_POS = 3
    IK_VEL = 4
    TYPING = 5

class DebugArmPublisher(Node):
    pos_pub: Publisher
    vel_pub: Publisher
    typing_client: ActionClient
    mode: Mode
    def __init__(self):
        super().__init__("debug_arm_publisher")

        self.mode = Mode.TYPING

        self.thr_pub = self.create_publisher(Throttle, "arm_thr_cmd", 1)
        self.pos_pub = self.create_publisher(Position, "arm_pos_cmd", 1)
        self.vel_pub = self.create_publisher(Velocity, "arm_vel_cmd", 1)
        self.typing_client = ActionClient(self, TypingPosition, "typing_ik")
        hz = 30.0

        match self.mode:
            case Mode.THR:
                self.thr_timer = self.create_timer(1.0/hz, self.throttle_timer_callback)
            case Mode.POS:
                self.pos_timer = self.create_timer(1.0/hz, self.pos_timer_callback)
            case Mode.TYPING:
                typing_future = self.typing_send_goal()
                rclpy.spin_until_future_complete(self, typing_future)


    def throttle_timer_callback(self):
        header = Header(stamp = self.get_clock().now().to_msg(), frame_id = "base_link")

        joint_names = ["joint_a", "gripper"]
        joint_throttles = [-1.0, 0.0]
        
        arm_thr_cmd = Throttle(header=header, names=joint_names, throttles=joint_throttles)
        self.thr_pub.publish(arm_thr_cmd)

    def pos_timer_callback(self):
        header = Header(stamp = self.get_clock().now().to_msg(), frame_id = "base_link")

        joint_names = ["joint_a", "gripper"]
        joint_positions = [0.0, 0.0]
        
        arm_pos_cmd = Position(header=header, names=joint_names, positions=joint_positions)
        self.pos_pub.publish(arm_pos_cmd)

    def vel_timer_callback(self):
        pass

    def typing_send_goal(self):
        typing_goal = TypingPosition.Goal(x=0.05, y=0.05)
        self.typing_client.wait_for_server()
        return self.typing_client.send_goal_async(typing_goal)
    



if __name__ == "__main__":
    try:
        rclpy.init(args=sys.argv)
        rclpy.spin(DebugArmPublisher())
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
