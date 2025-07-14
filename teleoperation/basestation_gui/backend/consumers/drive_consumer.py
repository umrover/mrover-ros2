import json
import traceback
from typing import Any, Type

from channels.generic.websocket import JsonWebsocketConsumer
from rosidl_runtime_py.convert import message_to_ordereddict

import threading
from rclpy.executors import MultiThreadedExecutor
from backend.consumers.init_node import get_node, get_context

from backend.drive_controls import send_joystick_twist
from backend.input import DeviceInputs
from mrover.msg import ControllerState

LOCALIZATION_INFO_HZ = 10

class DriveConsumer(JsonWebsocketConsumer):
    subscribers = []
    timers = []

    def connect(self) -> None:
        self.accept()

        self.node = get_node()
        self.ros_context = get_context()

        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()

        # Forwards ROS topic to GUI
        self.forward_ros_topic("/drive_left_controller_data", ControllerState, "drive_left_state")
        self.forward_ros_topic("/drive_right_controller_data", ControllerState, "drive_right_state")
        self.forward_ros_topic("/drive_controller_data", ControllerState, "drive_state")

        # Services

    def disconnect(self, close_code) -> None:
        try:
            for subscriber in self.subscribers:
                self.node.destroy_subscription(subscriber)
            self.subscribers.clear()
            self.timers.clear()

            if self.ros_thread.is_alive():
                self.ros_thread.join(timeout=1)
        except Exception as e:
            print(f"Exception during disconnect cleanup: {e}")

    def ros_spin(self) -> None:
        executor = MultiThreadedExecutor(context=self.ros_context)
        executor.add_node(self.node)
        try:
            executor.spin()
        except Exception as e:
            print(f"Exception in ROS spin: {e}")

    def forward_ros_topic(self, topic_name: str, topic_type: Type, gui_msg_type: str) -> None:
        """
        Subscribes to a ROS topic and forwards messages to the GUI as JSON

        @param topic_name:      ROS topic name
        @param topic_type:      ROS message type
        @param gui_msg_type:    String to identify the message type in the GUI
        """

        def callback(ros_message: Any):
            self.send_message_as_json({"type": gui_msg_type, **message_to_ordereddict(ros_message)})

        self.subscribers.append(self.node.create_subscription(topic_type, topic_name, callback, qos_profile=1))

    def send_message_as_json(self, msg: dict):
        try:
            self.send(text_data=json.dumps(msg))
        except Exception as e:
            self.node.get_logger().warning(f"Failed to send message: {e}")

    def receive(self, text_data=None, bytes_data=None, **kwargs) -> None:
        """
        Callback function when a message is received in the Websocket

        @param text_data:   Stringfied JSON message
        """

        if text_data is None:
            self.node.get_logger().warning("Expecting text but received binary on GUI websocket...")

        try:
            message = json.loads(text_data)
        except json.JSONDecodeError as e:
            self.node.get_logger().warning(f"Failed to decode JSON: {e}")

        try:
            match message:
                case{
                    "type": "joystick",
                    "axes": axes, 
                    "buttons": buttons
                }:
                    device_input = DeviceInputs(axes, buttons)
                    send_joystick_twist(device_input, self.joystick_twist_pub)

                case _:
                    self.node.get_logger().warning(f"Unhandled message on drive: {message}")
        except:
            self.node.get_logger().error(f"Failed to handle message: {message}")
            self.node.get_logger().error(traceback.format_exc())