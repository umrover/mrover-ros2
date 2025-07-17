import json
import traceback
from typing import Any, Type
import asyncio
import rclpy
import numpy as np
import cv2

from channels.generic.websocket import AsyncJsonWebsocketConsumer
from asgiref.sync import sync_to_async
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

from backend.consumers.init_node import get_node, get_context
from backend.input import DeviceInputs
from backend.mast_controls import send_mast_controls
from sensor_msgs.msg import Image
from mrover.msg import Throttle
from mrover.srv import PanoramaStart, PanoramaEnd


class MastConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self) -> None:
        await self.accept()

        self.node = get_node()
        self.ros_context = get_context()
        self.subscribers = []

        self.mast_gimbal_pub = self.node.create_publisher(Throttle, "/mast_gimbal_throttle_cmd", 1)
        self.pano_start_srv = self.node.create_client(PanoramaStart, "/panorama/start")
        self.pano_end_srv = self.node.create_client(PanoramaEnd, "/panorama/end")

        self.ros_task = asyncio.to_thread(self.ros_spin)

    async def disconnect(self, close_code) -> None:
        print("MastConsumer disconnecting...")
        try:
            for subscriber in self.subscribers:
                await sync_to_async(self.node.destroy_subscription)(subscriber)
            self.subscribers.clear()

            if self.ros_context.is_valid():
                rclpy.shutdown(context=self.ros_context)

            await self.ros_task
            print("ROS spin task for MastConsumer finished.")
        except Exception as e:
            print(f"Exception during MastConsumer disconnect: {e}")

    def ros_spin(self) -> None:
        executor = MultiThreadedExecutor(context=self.ros_context)
        executor.add_node(self.node)
        try:
            executor.spin()
        except (ExternalShutdownException, RuntimeError):
            print("ROS executor for MastConsumer has been shut down.")
        finally:
            print("ROS spin loop for MastConsumer exited.")

    async def handle_pano_stop(self):
        try:
            self.node.get_logger().info("Requesting panorama image...")
            future = self.pano_end_srv.call_async(PanoramaEnd.Request())
            result = await future

            if result is not None and result.success:
                img_msg: Image = result.img
                img_np = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                    img_msg.height, img_msg.width, -1 # Use -1 for automatic channel detection
                )
                timestamp = f"{self.node.get_clock().now().seconds_nanoseconds()[0]}"
                filename = f"../../data/{timestamp}_panorama.png"
                
                # Run blocking file I/O in a separate thread
                await asyncio.to_thread(cv2.imwrite, filename, img_np)
                self.node.get_logger().info(f"Image saved to {filename}")
            else:
                self.node.get_logger().error("Panorama service call failed or returned no image.")
        except Exception as e:
            self.node.get_logger().error(f"Error handling panorama response: {e}\n{traceback.format_exc()}")

    async def receive_json(self, content, **kwargs) -> None:
        message = content
        try:
            match message:
                case {"type": "mast_keyboard", "axes": axes, "buttons": buttons}:
                    device_input = DeviceInputs(axes, buttons)
                    await asyncio.to_thread(send_mast_controls, device_input, self.mast_gimbal_pub)
                
                case {"type": "pano", "action": "start"}:
                    self.node.get_logger().info("Panorama start requested.")
                    await self.pano_start_srv.call_async(PanoramaStart.Request())
                
                case {"type": "pano", "action": "stop"}:
                    await self.handle_pano_stop()
                    
                case _:
                    self.node.get_logger().warning(f"Unhandled message on mast: {message}")
        except Exception:
            self.node.get_logger().error(f"Failed to handle message: {message}")
            self.node.get_logger().error(traceback.format_exc())