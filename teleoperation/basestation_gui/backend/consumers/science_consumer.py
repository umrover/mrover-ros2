import json
import traceback
from typing import Any, Type
import asyncio
import rclpy

from channels.generic.websocket import AsyncJsonWebsocketConsumer
from asgiref.sync import sync_to_async
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

from backend.consumers.init_node import get_node, get_context
from sensor_msgs.msg import Temperature, RelativeHumidity
from mrover.msg import (
    LED,
    HeaterData,
    ScienceThermistors,
    Oxygen,
    Methane,
    UV,
)
from mrover.srv import EnableBool, ServoSetPos
from std_msgs.msg import Float32

heater_names: list[str] = ["a0", "a1", "b0", "b1"]


class ScienceConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self) -> None:
        await self.accept()

        self.node = get_node()
        self.ros_context = get_context()
        self.subscribers = []

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
        self.heater_services = [self.node.create_client(EnableBool, f"/science_enable_heater_{name}") for name in heater_names]
        self.white_leds_services = [self.node.create_client(EnableBool, f"/science_enable_white_led_{site}") for site in ["a", "b"]]

        self.ros_task = asyncio.to_thread(self.ros_spin)

    async def disconnect(self, close_code) -> None:
        print("ScienceConsumer disconnecting...")
        try:
            for subscriber in self.subscribers:
                await sync_to_async(self.node.destroy_subscription)(subscriber)
            self.subscribers.clear()

            if self.ros_context.is_valid():
                rclpy.shutdown(context=self.ros_context)

            await self.ros_task
            print("ROS spin task for ScienceConsumer finished.")
        except Exception as e:
            print(f"Exception during ScienceConsumer disconnect: {e}")

    def ros_spin(self) -> None:
        executor = MultiThreadedExecutor(context=self.ros_context)
        executor.add_node(self.node)
        try:
            executor.spin()
        except (ExternalShutdownException, RuntimeError):
            print("ROS executor for ScienceConsumer has been shut down.")
        finally:
            print("ROS spin loop for ScienceConsumer exited.")

    async def forward_ros_topic(self, topic_name: str, topic_type: Type, gui_msg_type: str) -> None:
        loop = asyncio.get_running_loop()

        def callback(ros_message: Any):
            data_to_send = {"type": gui_msg_type, **message_to_ordereddict(ros_message)}
            asyncio.run_coroutine_threadsafe(self.send_json(data_to_send), loop)

        sub = await sync_to_async(self.node.create_subscription)(
            topic_type, topic_name, callback, qos_profile=1
        )
        self.subscribers.append(sub)

    async def receive_json(self, content, **kwargs) -> None:
        message = content
        try:
            match message:
                case {"type": "heater_enable", "enable": e, "heater": heater}:
                    req = EnableBool.Request(enable=e)
                    await self.heater_services[heater_names.index(heater)].call_async(req)

                case {"type": "set_gear_diff_pos", "position": position, "isCCW": isCCW}:
                    req = ServoSetPos.Request(position=float(position), is_counterclockwise=isCCW)
                    await self.gear_diff_set_pos_srv.call_async(req)

                case {"type": "auto_shutoff", "shutoff": shutoff}:
                    req = EnableBool.Request(enable=shutoff)
                    await self.auto_shutoff_service.call_async(req)

                case {"type": "white_leds", "site": site, "enable": e}:
                    req = EnableBool.Request(enable=e)
                    await self.white_leds_services[site].call_async(req)

                case {"type": "ls_toggle", "enable": e}:
                    req = EnableBool.Request(enable=e)
                    await self.sa_enable_switch_srv.call_async(req)

                case _:
                    self.node.get_logger().warning(f"Unhandled message on science: {message}")
        except Exception:
            self.node.get_logger().error(f"Failed to handle message: {message}")
            self.node.get_logger().error(traceback.format_exc())