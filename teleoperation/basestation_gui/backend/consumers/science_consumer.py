import traceback
from typing import Any, Type
import asyncio

from channels.generic.websocket import AsyncJsonWebsocketConsumer
from asgiref.sync import sync_to_async
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.qos import qos_profile_sensor_data

from backend.consumers.ros_manager import get_node
from sensor_msgs.msg import Temperature, RelativeHumidity
from mrover.msg import (
    LED,
    HeaterData,
    ScienceThermistors,
    Oxygen,
    Methane,
    UV,
)
from mrover.srv import (
    EnableBool,
    ServoSetPos
)
from std_msgs.msg import Float32

heater_names: list[str] = ["a0", "a1", "b0", "b1"]

class ScienceConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self) -> None:
        await self.accept()

        self.node = get_node()
        self.subscribers = []
        self.timers = []

        await self.forward_ros_topic("/led", LED, "led")
        await self.forward_ros_topic("/science_thermistors", ScienceThermistors, "thermistors")
        await self.forward_ros_topic("/science_heater_state", HeaterData, "heater_states")
        await self.forward_ros_topic("/science_oxygen_data", Oxygen, "oxygen")
        await self.forward_ros_topic("/science_methane_data", Methane, "methane")
        await self.forward_ros_topic("/science_uv_data", UV, "uv")
        await self.forward_ros_topic("/science_temperature_data", Temperature, "temperature")
        await self.forward_ros_topic("/science_humidity_data", RelativeHumidity, "humidity")
        await self.forward_ros_topic("/sa_gear_diff_position", Float32, "hexhub_site")

        self.auto_shutoff_service = self.node.create_client(EnableBool, "/science_change_heater_auto_shutoff_state")
        self.sa_enable_switch_srv = self.node.create_client(EnableBool, "/sa_enable_limit_switch_sensor_actuator")
        self.gear_diff_set_pos_srv = self.node.create_client(ServoSetPos, "/sa_gear_diff_set_position")
        
        self.heater_services = [
            self.node.create_client(EnableBool, f"/science_enable_heater_{name}") for name in heater_names
        ]
        self.white_leds_services = [
            self.node.create_client(EnableBool, f"/science_enable_white_led_{site}") for site in ["a", "b"]
        ]

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
                case {"type": "heater_enable", "enable": e, "heater": heater}:
                    request = EnableBool.Request()
                    request.enable = e
                    client = self.heater_services[heater_names.index(heater)]
                    await client.call_async(request)

                case {"type": "set_gear_diff_pos", "position": position, "isCCW": isCCW}:
                    request = ServoSetPos.Request()
                    request.position = float(position)
                    request.is_counterclockwise = isCCW
                    await self.gear_diff_set_pos_srv.call_async(request)

                case {"type": "auto_shutoff", "shutoff": shutoff}:
                    request = EnableBool.Request()
                    request.enable = shutoff
                    await self.auto_shutoff_service.call_async(request)

                case {"type": "white_leds", "site": site, "enable": e}:
                    request = EnableBool.Request()
                    request.enable = e
                    client = self.white_leds_services[site]
                    await client.call_async(request)

                case {"type": "ls_toggle", "enable": e}:
                    request = EnableBool.Request()
                    request.enable = e
                    await self.sa_enable_switch_srv.call_async(request)

                case _:
                    self.node.get_logger().warning(f"Unhandled message on science: {content}")
        except Exception:
            self.node.get_logger().error(f"Failed to handle message: {content}")
            self.node.get_logger().error(traceback.format_exc())