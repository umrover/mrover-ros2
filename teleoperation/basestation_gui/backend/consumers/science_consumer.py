import traceback
from typing import Any, Type
import asyncio

from channels.generic.websocket import AsyncJsonWebsocketConsumer
from asgiref.sync import sync_to_async
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.qos import qos_profile_sensor_data

from backend.consumers.ros_manager import get_node
from backend.sp_controls import send_sp_controls
from backend.input import DeviceInputs
from sensor_msgs.msg import Temperature, RelativeHumidity, JointState
from mrover.msg import (
    LED,
    Oxygen,
    UV,
    Throttle,
    ControllerState,
)
from std_msgs.msg import Float32

class ScienceConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self) -> None:
        await self.accept()

        self.node = get_node()
        self.subscribers = []
        self.timers = []

        self.sp_thr_pub = self.node.create_publisher(Throttle, "/sp_throttle_cmd", 1)

        await self.forward_ros_topic("/led", LED, "led")
        await self.forward_ros_topic("/science_oxygen_data", Oxygen, "oxygen")
        await self.forward_ros_topic("/science_uv_data", UV, "uv")
        await self.forward_ros_topic("/science_temperature_data", Temperature, "temperature")
        await self.forward_ros_topic("/science_humidity_data", RelativeHumidity, "humidity")
        await self.forward_ros_topic("/sp_joint_state", JointState, "sp_joint_state")
        await self.forward_ros_topic("/sp_controller_state", ControllerState, "sp_state") # not used yet

    async def disconnect(self, close_code) -> None:
        """
        Thread-safe disconnect method that schedules resource destruction
        on the ROS executor thread to prevent race conditions.
        """
        print(f"Scheduling cleanup for disconnected client {self.channel_name}...")

        def cleanup_ros_resources():
            """This function will be executed safely by the ROS thread."""
            # Clean up all ROS entities created in connect()
            for sub in self.subscribers:
                self.node.destroy_subscription(sub)
            self.subscribers.clear()

            for timer in self.timers:
                self.node.destroy_timer(timer)
            self.timers.clear()

            print(f"ROS resources for {self.channel_name} have been cleaned up.")

        try:
            # Add the entire cleanup function as a single callback to the executor
            self.executor.add_callback(cleanup_ros_resources)
        except Exception as e:
            print(f"Exception while scheduling disconnect cleanup: {e}")

    async def forward_ros_topic(self, topic_name: str, topic_type: Type, gui_msg_type: str) -> None:
        loop = asyncio.get_running_loop()

        def callback(ros_message: Any):
            data_to_send = {"type": gui_msg_type, **message_to_ordereddict(ros_message)}
            asyncio.run_coroutine_threadsafe(self.send_json(data_to_send), loop)

        sub = await sync_to_async(self.node.create_subscription)(
            topic_type, topic_name, callback, qos_profile=qos_profile_sensor_data
        )
        self.subscribers.append(sub)

    async def receive_json(self, content: dict, **kwargs) -> None:
        try:
            match content:
                case {
                    "type": "sp_controller",
                    "axes": axes,
                    "buttons": buttons,
                }:
                    device_input = DeviceInputs(axes, buttons)
                    send_sp_controls(device_input, self.sp_thr_pub)

                case _:
                    self.node.get_logger().warning(f"Science WebSocket received unexpected message: {content}")
        except Exception:
            self.node.get_logger().error(f"Failed to handle message: {content}")
            self.node.get_logger().error(traceback.format_exc())