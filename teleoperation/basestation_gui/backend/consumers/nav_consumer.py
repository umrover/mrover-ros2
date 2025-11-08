import json
import traceback
from typing import Any, Type
import asyncio

from channels.generic.websocket import AsyncJsonWebsocketConsumer
from asgiref.sync import sync_to_async
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.qos import qos_profile_sensor_data

from lie import SE3
import tf2_ros
from tf2_ros.buffer import Buffer

from backend.consumers.ros_manager import get_node
from sensor_msgs.msg import NavSatFix
from mrover.msg import StateMachineStateUpdate

LOCALIZATION_INFO_HZ = 10

class NavConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self) -> None:
        await self.accept()

        self.node = get_node()
        self.subscribers = []
        self.timers = []
        self.loop = asyncio.get_running_loop()

        await self.forward_ros_topic("/nav_state", StateMachineStateUpdate, "nav_state")
        await self.forward_ros_topic("/gps/fix", NavSatFix, "gps_fix")
        await self.forward_ros_topic("basestation/position", NavSatFix, "basestation_position")
        await self.forward_ros_topic("/drone_odometry", NavSatFix, "drone_waypoint")

        self.buffer = Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buffer, self.node, spin_thread=False)

        timer = self.node.create_timer(1.0 / LOCALIZATION_INFO_HZ, self.send_localization_callback)
        self.timers.append(timer)

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
        def callback(ros_message: Any):
            data_to_send = {"type": gui_msg_type, **message_to_ordereddict(ros_message)}
            asyncio.run_coroutine_threadsafe(self.send_json(data_to_send), self.loop)

        sub = await sync_to_async(self.node.create_subscription)(
            topic_type, topic_name, callback, qos_profile=qos_profile_sensor_data
        )
        self.subscribers.append(sub)

    def send_localization_callback(self):
        try:
            base_link_in_map = SE3.from_tf_tree(self.buffer, "map", "base_link")
            quat = base_link_in_map.quat().tolist()
            data_to_send = {
                "type": "orientation",
                "orientation": {
                    "x": quat[0],
                    "y": quat[1],
                    "z": quat[2],
                    "w": quat[3],
                },
            }
            asyncio.run_coroutine_threadsafe(self.send_json(data_to_send), self.loop)
        except Exception:
            # Errors are expected if localization isn't running, so we can pass silently.
            pass

    async def receive_json(self, content: dict, **kwargs) -> None:
        try:
            match content:
                case _:
                    self.node.get_logger().warning(f"Unhandled message on nav: {content}")
        except Exception:
            self.node.get_logger().error(f"Failed to handle message: {content}")
            self.node.get_logger().error(traceback.format_exc())