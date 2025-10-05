import traceback
from typing import Any, Type
import asyncio

from channels.generic.websocket import AsyncJsonWebsocketConsumer
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.qos import qos_profile_sensor_data

from backend.consumers.ros_manager import get_node
from backend.drive_controls import send_joystick_twist
from backend.input import DeviceInputs
from mrover.msg import ControllerState
from geometry_msgs.msg import Twist


class DriveConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self) -> None:
        await self.accept()

        self.node = get_node()
        self.subscribers = []
        self.timers = []

        self.joystick_twist_pub = self.node.create_publisher(Twist, "/joystick_cmd_vel", 1)

        await self.forward_ros_topic("/drive_left_controller_data", ControllerState, "drive_left_state")
        await self.forward_ros_topic("/drive_right_controller_data", ControllerState, "drive_right_state")
        await self.forward_ros_topic("/drive_controller_data", ControllerState, "drive_state")

    async def disconnect(self, close_code) -> None:
        """Clean up ROS resources on disconnect."""
        print(f"Cleaning up resources for disconnected client {self.channel_name}...")

        for sub in self.subscribers:
            self.node.destroy_subscription(sub)
        self.subscribers.clear()

        for timer in self.timers:
            self.node.destroy_timer(timer)
        self.timers.clear()

        print(f"ROS resources for {self.channel_name} have been cleaned up.")

    async def forward_ros_topic(self, topic_name: str, topic_type: Type, gui_msg_type: str) -> None:
        def callback(ros_message: Any):
            data_to_send = {"type": gui_msg_type, **message_to_ordereddict(ros_message)}
            asyncio.create_task(self.send_json(data_to_send))

        sub = self.node.create_subscription(
            topic_type, topic_name, callback, qos_profile=qos_profile_sensor_data
        )
        self.subscribers.append(sub)

    async def receive_json(self, content: dict, **kwargs) -> None:
        try:
            match content:
                case {
                    "type": "joystick",
                    "axes": axes,
                    "buttons": buttons
                }:
                    device_input = DeviceInputs(axes, buttons)
                    send_joystick_twist(device_input, self.joystick_twist_pub)

                case _:
                    self.node.get_logger().warning(f"Unhandled message on drive: {content}")
        except Exception:
            self.node.get_logger().error(f"Failed to handle message: {content}")
            self.node.get_logger().error(traceback.format_exc())