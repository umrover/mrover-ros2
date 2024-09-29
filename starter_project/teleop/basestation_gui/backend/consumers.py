import json
import traceback
from typing import Any, Type

import yaml
from channels.generic.websocket import JsonWebsocketConsumer

import rclpy
from rclpy.subscription import Subscription
from rclpy.node import Node

import tf2_ros
import numpy as np
from backend.drive_controls import send_joystick_twist
from backend.input import DeviceInputs
from mrover.msg import WheelCmd
from geometry_msgs.msg import Twist

node = rclpy.create_node('teleoperation')

class GUIConsumer(JsonWebsocketConsumer):
    subscribers: list[Subscription] = []
    node: Node = None

    def connect(self) -> None:
        self.accept()

        # node = rclpy.create_node('teleoperation')

        ########################################################################################
        # Use self.forward_ros_topic when you want to get data from a ROS topic to a GUI
        # without needing any modifications done on it. For instance, reading motor output.
        ########################################################################################
        self.forward_ros_topic("/wheel_cmd", WheelCmd, "wheel_cmd")
        rclpy.spin(node)

    def disconnect(self, close_code) -> None:
        for subscriber in self.subscribers:
            node.destroy_subscription(subscriber)

    def forward_ros_topic(self, topic_name: str, topic_type: Type, gui_msg_type: str) -> None:
        """
        Subscribes to a ROS topic and forwards messages to the GUI as JSON

        @param topic_name:      ROS topic name
        @param topic_type:      ROS message type
        @param gui_msg_type:    String to identify the message type in the GUI
        """

        def callback(ros_message: Any):
            # Formatting a ROS message as a string outputs YAML
            # Parse it back into a dictionary, so we can send it as JSON
            node.get_logger().error(f"{ros_message}")
            self.send_message_as_json({"type": gui_msg_type})
            # self.send_message_as_json({"type": gui_msg_type, **yaml.safe_load(str(ros_message))})
        self.subscribers.append(node.create_subscription(topic_type, topic_name , callback, qos_profile=1))
        node.get_logger().error(f"{self.subscribers[0].callback}")

    def send_message_as_json(self, msg: dict):
        try:
            self.send(text_data=json.dumps(msg))
        except Exception as e:
            node.get_logger().warning(f"Failed to send message: {e}")


    def receive(self, text_data=None, bytes_data=None, **kwargs) -> None:
        """
        Callback function when a message is received in the Websocket

        @param text_data:   Stringfied JSON message
        """

        if text_data is None:
            node.get_logger().warning("Expecting text but received binary on GUI websocket...")
            return

        try:
            message = json.loads(text_data)
        except json.JSONDecodeError as e:
            node.get_logger().warning(f"Failed to decode JSON: {e}")
            return

        try:
            match message:
                case {
                    "type": "joystick",
                    "axes": axes,
                    "buttons": buttons,
                }:
                    device_input = DeviceInputs(axes, buttons)
                    send_joystick_twist(device_input)
                case _:
                    node.get_logger().warning(f"Unhandled message: {message}")

        except:
            node.get_logger().error(f"Failed to handle message: {message}")
            node.get_logger().error(traceback.format_exc())