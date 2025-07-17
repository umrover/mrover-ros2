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
from backend.waypoints import (
    get_auton_waypoint_list,
    get_basic_waypoint_list,
    get_current_auton_course,
    get_current_basic_course,
    save_auton_waypoint_list,
    save_basic_waypoint_list,
    save_current_auton_course,
    save_current_basic_course,
    delete_auton_waypoint_from_course,
)


class WaypointsConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self) -> None:
        await self.accept()

        self.node = get_node()
        self.ros_context = get_context()

        self.ros_task = asyncio.to_thread(self.ros_spin)

    async def disconnect(self, close_code) -> None:
        print("WaypointsConsumer disconnecting...")
        try:
            if self.ros_context.is_valid():
                rclpy.shutdown(context=self.ros_context)

            await self.ros_task
            print("ROS spin task for WaypointsConsumer finished.")
        except Exception as e:
            print(f"Exception during WaypointsConsumer disconnect: {e}")

    def ros_spin(self) -> None:
        executor = MultiThreadedExecutor(context=self.ros_context)
        executor.add_node(self.node)
        try:
            executor.spin()
        except (ExternalShutdownException, RuntimeError):
            print("ROS executor for WaypointsConsumer has been shut down.")
        finally:
            print("ROS spin loop for WaypointsConsumer exited.")

    async def receive_json(self, content, **kwargs) -> None:
        message = content
        try:
            match message:
                case {"type": "save_auton_waypoint_list", "data": waypoints}:
                    await asyncio.to_thread(save_auton_waypoint_list, waypoints)

                case {"type": "save_basic_waypoint_list", "data": waypoints}:
                    await asyncio.to_thread(save_basic_waypoint_list, waypoints)

                case {"type": "save_current_auton_course", "data": waypoints}:
                    await asyncio.to_thread(save_current_auton_course, waypoints)

                case {"type": "save_current_basic_course", "data": waypoints}:
                    await asyncio.to_thread(save_current_basic_course, waypoints)

                case {"type": "delete_auton_waypoint_from_course", "data": waypoint}:
                    await asyncio.to_thread(delete_auton_waypoint_from_course, waypoint)

                case {"type": "get_basic_waypoint_list"}:
                    data = await asyncio.to_thread(get_basic_waypoint_list)
                    await self.send_json({"type": "get_basic_waypoint_list", "data": data})

                case {"type": "get_auton_waypoint_list"}:
                    data = await asyncio.to_thread(get_auton_waypoint_list)
                    await self.send_json({"type": "get_auton_waypoint_list", "data": data})

                case {"type": "get_current_basic_course"}:
                    data = await asyncio.to_thread(get_current_basic_course)
                    await self.send_json({"type": "get_current_basic_course", "data": data})

                case {"type": "get_current_auton_course"}:
                    data = await asyncio.to_thread(get_current_auton_course)
                    await self.send_json({"type": "get_current_auton_course", "data": data})
                    
                case {"type": "debug", "timestamp": ts}:
                    self.node.get_logger().debug(f"debug message received by consumer, {ts}")

                case _:
                    self.node.get_logger().warning(f"Unhandled message on waypoints: {message}")
        except Exception:
            self.node.get_logger().error(f"Failed to handle message: {message}")
            self.node.get_logger().error(traceback.format_exc())