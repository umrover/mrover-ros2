#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mrover.msg import ControllerState


class FakeControllerStatePublisher(Node):

    def __init__(self):
        super().__init__("fake_controller_state_publisher")

        self.arm_publisher = self.create_publisher(ControllerState, "/arm_controller_state", 10)

        self.drive_publisher = self.create_publisher(ControllerState, "/drive_controller_data", 10)

        self.timer = self.create_timer(0.1, self.publish_messages)

        self.get_logger().info("Publishing fake controller state data at 10 Hz")

    def publish_messages(self):
        arm_msg = ControllerState()
        arm_msg.name = ["shoulder", "elbow", "wrist", "gripper"]
        arm_msg.state = ["Armed", "Armed", "Armed", "Error"]
        arm_msg.error = ["", "", "", "Encoder fault"]
        arm_msg.limit_hit = [0, 1, 0, 3]

        drive_msg = ControllerState()
        drive_msg.name = ["FL", "FR", "ML", "MR", "BL", "BR"]
        drive_msg.state = ["Armed", "Armed", "Armed", "Armed", "Armed", "Armed"]
        drive_msg.error = ["", "", "", "", "", ""]
        drive_msg.limit_hit = [0, 0, 0, 0, 0, 0]

        self.arm_publisher.publish(arm_msg)
        self.drive_publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    publisher = FakeControllerStatePublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
