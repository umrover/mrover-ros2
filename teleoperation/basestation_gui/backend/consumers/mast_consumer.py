import json
import traceback
from typing import Any, Type
import asyncio

from channels.generic.websocket import AsyncJsonWebsocketConsumer
from asgiref.sync import sync_to_async
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import cv2

from backend.consumers.ros_manager import get_node
from backend.input import DeviceInputs
from backend.mast_controls import send_mast_controls
from sensor_msgs.msg import Image
from mrover.msg import Throttle
from mrover.srv import (
    PanoramaStart,
    PanoramaEnd,
)


class MastConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self) -> None:
        await self.accept()

        self.node = get_node()
        self.subscribers = []
        self.timers = []

        self.mast_gimbal_pub = self.node.create_publisher(Throttle, "/mast_gimbal_throttle_cmd", 1)

        self.pano_start_srv = self.node.create_client(PanoramaStart, "/panorama/start")
        self.pano_end_srv = self.node.create_client(PanoramaEnd, "/panorama/end")

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

    async def start_stop_pano(self, action: str) -> None:
        if action == "start":
            self.node.get_logger().info("Pano start service called.")
            await self.pano_start_srv.call_async(PanoramaStart.Request())
        else:
            self.node.get_logger().info("Pano end service called, awaiting result...")
            try:
                result = await self.pano_end_srv.call_async(PanoramaEnd.Request())
                
                if result is not None and result.success:
                    self.node.get_logger().info("Panorama successful, processing image.")
                    
                    def save_image_sync(img_msg: Image):
                        img_np = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                            img_msg.height, img_msg.width, -1 
                        )
                        # Ensure it's BGR for OpenCV if it's RGBA or BGRA
                        if img_np.shape[2] == 4:
                            img_np = cv2.cvtColor(img_np, cv2.COLOR_RGBA2BGR)
                        
                        timestamp = self.node.get_clock().now().nanoseconds
                        filename = f"../../data/{timestamp}_panorama.png"
                        cv2.imwrite(filename, img_np)
                        self.node.get_logger().info(f"Image saved to {filename}")

                    await sync_to_async(save_image_sync)(result.img)
                else:
                    self.node.get_logger().warn("Panorama service reported no success or null result.")

            except Exception as e:
                self.node.get_logger().error(f"Panorama service call failed: {e}")

    async def receive_json(self, content: dict, **kwargs) -> None:
        try:
            match content:
                case {
                    "type": "mast_keyboard",
                    "axes": axes,
                    "buttons": buttons,
                }:
                    device_input = DeviceInputs(axes, buttons)
                    send_mast_controls(device_input, self.mast_gimbal_pub)
                case {"type": "pano", "action": a}:
                    await self.start_stop_pano(a)
                    
                case _:
                    self.node.get_logger().warning(f"Unhandled message on mast: {content}")
        except Exception:
            self.node.get_logger().error(f"Failed to handle message: {content}")
            self.node.get_logger().error(traceback.format_exc())