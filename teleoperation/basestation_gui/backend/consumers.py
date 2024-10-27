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
from backend.models import AutonTyping
from mrover.msg import WheelCmd
from geometry_msgs.msg import Twist

import threading

import logging
logger = logging.getLogger('django')

node = rclpy.create_node('teleoperation')
thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
thread.start()

class GUIConsumer(JsonWebsocketConsumer):
    subscribers = []
    
    def connect(self) -> None:
        self.forward_ros_topic("/angle", , "wheel_cmd") # angle message type? need to send to front end
        # create a publisher
        self.accept()

    def forward_ros_topic(self, topic_name: str, topic_type: Type, gui_msg_type: str) -> None:
        """
        Subscribes to a ROS topic and forwards messages to the GUI as JSON

        @param topic_name:      ROS topic name
        @param topic_type:      ROS message type
        @param gui_msg_type:    String to identify the message type in the GUI
        """

        def ros_message_to_dict(msg):
            if hasattr(msg, '__slots__'):
                msg_dict = {}
                for slot in msg.__slots__:
                    value = getattr(msg, slot)
                    # Recursively convert ROS messages and remove leading underscores from the slot names
                    key = slot.lstrip('_')
                    msg_dict[key] = ros_message_to_dict(value)
                return msg_dict
            return msg
            
        def callback(ros_message: Any):
            # Formatting a ROS message as a string outputs YAML
            # Parse it back into a dictionary, so we can send it as JSON
            self.send_message_as_json({"type": gui_msg_type, **ros_message_to_dict(ros_message)})
        self.subscribers.append(node.create_subscription(topic_type, topic_name , callback, qos_profile=1))

    def send_message_as_json(self, msg: dict):
        try:
            self.send(text_data=json.dumps(msg))
        except Exception as e:
            node.get_logger().warning(f"Failed to send message: {e}")

    def get_auton_typing_message(self) -> None:
        self.send_message_as_json(
            {
                "type": "get_auton_typing_message",
                "data": [
                    {"typingMessage": AutonTyping.objects.typingMessage}
                ],
            }
        )


    def receive(self, text_data=None, bytes_data=None, **kwargs) -> None:
        """
        Callback function when a message is received in the Websocket

        @param text_data:   Stringfied JSON message
        """

        if text_data is None:
            # node.get_logger().warning("Expecting text but received binary on GUI websocket...")
            return

        try:
            message = json.loads(text_data)
        except json.JSONDecodeError as e:
            # node.get_logger().warning(f"Failed to decode JSON: {e}")
            return

        try:
            match message:
                case {
                    "type": "joystick",
                    "axes": axes,
                    "buttons": buttons,
                }:
                    device_input = DeviceInputs(axes, buttons)
                    # pass in a publisher that you will define in connect function
                    send_joystick_twist(device_input)
                case _:
                    match message["type"]:
                        case "get_auton_typing_message":
                            self.get_auton_typing_message()

        except:
            # node.get_logger().error(f"Failed to handle message: {message}")
            # node.get_logger().error(traceback.format_exc())
            pass