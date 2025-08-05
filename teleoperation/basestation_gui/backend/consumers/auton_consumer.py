import traceback
from typing import Any, Type
import asyncio

from channels.generic.websocket import AsyncJsonWebsocketConsumer
from asgiref.sync import sync_to_async
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.qos import qos_profile_sensor_data

from backend.consumers.ros_manager import get_node

from mrover.srv import EnableAuton
from mrover.msg import (
    GPSWaypoint,
    WaypointType,
)
from std_srvs.srv import SetBool


class AutonConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self) -> None:
        await self.accept()

        self.node = get_node()
        self.subscribers = []
        self.timers = []

        self.enable_teleop_srv = self.node.create_client(SetBool, "/enable_teleop")
        self.enable_auton_srv = self.node.create_client(EnableAuton, "/enable_auton")

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

    async def send_auton_command(self, waypoints: list[dict], enabled: bool) -> None:
        try:
            request = EnableAuton.Request(
                enable=enabled,
                waypoints=[
                    GPSWaypoint(
                        tag_id=waypoint.get("tag_id", -1),
                        latitude_degrees=waypoint.get("latitude_degrees", 0.0),
                        longitude_degrees=waypoint.get("longitude_degrees", 0.0),
                        type=WaypointType(val=int(waypoint.get("type", 0))),
                        enable_costmap=waypoint.get("enable_costmap", True),
                    )
                    for waypoint in waypoints
                ],
            )
            await self.enable_auton_srv.call_async(request)
        except Exception:
            self.node.get_logger().error(f"Failed to send auton command: {traceback.format_exc()}")


    async def receive_json(self, content: dict, **kwargs) -> None:
        try:
            match content:
                case {"type": "auton_enable", "enabled": enabled, "waypoints": waypoints}:
                    await self.send_auton_command(waypoints, enabled)

                case {"type": "teleop_enable", "enabled": enabled}:
                    request = SetBool.Request()
                    request.data = enabled
                    await self.enable_teleop_srv.call_async(request)

                case _:
                    self.node.get_logger().warning(f"Unhandled message on auton: {content}")
        except Exception:
            self.node.get_logger().error(f"Failed to handle message: {content}")
            self.node.get_logger().error(traceback.format_exc())