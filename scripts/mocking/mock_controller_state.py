#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mrover.msg import ControllerState


class FakeControllerStatePublisher(Node):

    def __init__(self):
        super().__init__("fake_controller_state_publisher")

        self.arm_publisher = self.create_publisher(ControllerState, "/arm_controller_state", 10)

        self.drive_left_publisher = self.create_publisher(ControllerState, "/left_controller_state", 10)
        self.drive_right_publisher = self.create_publisher(ControllerState, "/right_controller_state", 10)

        self.timer = self.create_timer(0.1, self.publish_messages)

        self.get_logger().info("Publishing fake controller state data at 10 Hz")

    def publish_messages(self):
        arm_msg = ControllerState()
        arm_msg.names = ["shoulder", "elbow", "wrist", "gripper"]
        arm_msg.states = ["Armed", "Armed", "Armed", "Error"]
        arm_msg.errors = ["", "", "", "Encoder fault"]
        arm_msg.limits_hit = [0, 1, 0, 3]

        drive_left_msg = ControllerState()
        drive_left_msg.names = ["FL", "ML", "BL"]
        drive_left_msg.states = ["Armed", "Armed", "Armed"]
        drive_left_msg.errors = ["", "", ""]
        drive_left_msg.limits_hit = [0, 0, 0]

        drive_right_msg = ControllerState()
        drive_right_msg.names = ["FR", "MR", "BR"]
        drive_right_msg.states = ["Armed", "Armed", "Armed"]
        drive_right_msg.errors = ["", "", ""]
        drive_right_msg.limits_hit = [0, 0, 0]

        self.arm_publisher.publish(arm_msg)
        self.drive_left_publisher.publish(drive_left_msg)
        self.drive_right_publisher.publish(drive_right_msg)


def main(args=None):
    rclpy.init(args=args)
    publisher = FakeControllerStatePublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
