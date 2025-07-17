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
from sensor_msgs.msg import NavSatFix
from mrover.msg import StateMachineStateUpdate


class NavConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self) -> None:
        await self.accept()

        self.node = get_node()
        self.ros_context = get_context()
        self.subscribers = []

        await self.forward_ros_topic("/nav_state", StateMachineStateUpdate, "nav_state")
        await self.forward_ros_topic("/gps/fix", NavSatFix, "gps_fix")
        await self.forward_ros_topic("basestation/position", NavSatFix, "basestation_position")
        await self.forward_ros_topic("/drone_odometry", NavSatFix, "drone_waypoint")

        self.ros_task = asyncio.to_thread(self.ros_spin)

    async def disconnect(self, close_code) -> None:
        print("NavConsumer disconnecting...")
        try:
            for subscriber in self.subscribers:
                await sync_to_async(self.node.destroy_subscription)(subscriber)
            self.subscribers.clear()

            if self.ros_context.is_valid():
                rclpy.shutdown(context=self.ros_context)

            await self.ros_task
            print("ROS spin task for NavConsumer finished.")
        except Exception as e:
            print(f"Exception during NavConsumer disconnect: {e}")

    def ros_spin(self) -> None:
        executor = MultiThreadedExecutor(context=self.ros_context)
        executor.add_node(self.node)
        try:
            executor.spin()
        except (ExternalShutdownException, RuntimeError):
            print("ROS executor for NavConsumer has been shut down.")
        finally:
            print("ROS spin loop for NavConsumer exited.")

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
                case _:
                    self.node.get_logger().warning(f"Unhandled message on nav: {message}")
        except Exception:
            self.node.get_logger().error(f"Failed to handle message: {message}")
            self.node.get_logger().error(traceback.format_exc())