import json
import traceback
from typing import Any, Type
import asyncio
import rclpy

from channels.generic.websocket import AsyncJsonWebsocketConsumer
from asgiref.sync import sync_to_async

from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from backend.consumers.init_node import get_node, get_context
from mrover.srv import EnableAuton
from std_srvs.srv import SetBool


class AutonConsumer(AsyncJsonWebsocketConsumer):
    
    async def connect(self) -> None:
        await self.accept()

        self.node = get_node()
        self.ros_context = get_context()
        self.subscribers = []

        self.enable_teleop_srv = self.node.create_client(SetBool, "/enable_teleop")
        self.enable_auton_srv = self.node.create_client(EnableAuton, "/enable_auton")

        self.ros_task = asyncio.to_thread(self.ros_spin)

    async def disconnect(self, close_code) -> None:
        print("AutonConsumer disconnecting...")
        try:
            for subscriber in self.subscribers:
                await sync_to_async(self.node.destroy_subscription)(subscriber)
            self.subscribers.clear()

            if self.ros_context.is_valid():
                rclpy.shutdown(context=self.ros_context)

            await self.ros_task
            print("ROS spin task for AutonConsumer finished.")

        except Exception as e:
            print(f"Exception during disconnect cleanup: {e}")

    def ros_spin(self) -> None:
        """This synchronous function is run in a background thread."""
        executor = MultiThreadedExecutor(context=self.ros_context)
        executor.add_node(self.node)
        try:
            executor.spin()
        except (ExternalShutdownException, RuntimeError):
            print("ROS executor for AutonConsumer has been shut down.")
        finally:
            print("ROS spin loop for AutonConsumer exited.")

    async def receive_json(self, content, **kwargs) -> None:
        """Handles incoming JSON messages from the WebSocket."""
        message = content
        try:
            match message:
                case {"type": "auton_enable", "enabled": enabled, "waypoints": waypoints}:
                    await self.send_auton_command(waypoints, enabled)

                case {"type": "teleop_enable", "enabled": enabled}:
                    request = SetBool.Request(data=enabled)
                    future = self.enable_teleop_srv.call_async(request)
                    await future

                case _:
                    self.node.get_logger().warning(f"Unhandled message on auton: {message}")
        except Exception:
            self.node.get_logger().error(f"Failed to handle message: {message}")
            self.node.get_logger().error(traceback.format_exc())

    async def send_auton_command(self, waypoints: list, enabled: bool) -> None:
        """Creates and sends a request to the auton service."""
        try:
            request = EnableAuton.Request()
            request.waypoints = json.dumps(waypoints)
            request.enabled = enabled
            
            future = self.enable_auton_srv.call_async(request)
            response = await future
            self.node.get_logger().info(f"Auton service response: {response}")

        except Exception as e:
            self.node.get_logger().error(f"Failed to call auton service: {e}\n{traceback.format_exc()}")