import traceback
import asyncio

from channels.generic.websocket import AsyncJsonWebsocketConsumer
from asgiref.sync import sync_to_async

from backend.consumers.ros_manager import get_node
from backend.waypoints import (
    get_auton_waypoint_list,
    get_basic_waypoint_list,
    get_current_auton_course,
    get_current_basic_course,
    save_auton_waypoint_list,
    save_basic_waypoint_list,
    save_current_auton_course,
    save_current_basic_course,
    delete_auton_waypoint_from_course
)

class WaypointsConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self) -> None:
        await self.accept()
        self.node = get_node()

    async def disconnect(self, close_code) -> None:
        """
        Thread-safe disconnect method that schedules resource destruction
        on the ROS executor thread to prevent race conditions.
        """
        print(f"Scheduling cleanup for disconnected client {self.channel_name}...")

    async def receive_json(self, content: dict, **kwargs) -> None:
        try:
            match content:
                case {"type": "save_auton_waypoint_list", "data": waypoints}:
                    await sync_to_async(save_auton_waypoint_list)(waypoints)

                case {"type": "save_basic_waypoint_list", "data": waypoints}:
                    await sync_to_async(save_basic_waypoint_list)(waypoints)

                case {"type": "save_current_auton_course", "data": waypoints}:
                    await sync_to_async(save_current_auton_course)(waypoints)

                case {"type": "save_current_basic_course", "data": waypoints}:
                    await sync_to_async(save_current_basic_course)(waypoints)

                case {"type": "delete_auton_waypoint_from_course", "data": waypoint}:
                    await sync_to_async(delete_auton_waypoint_from_course)(waypoint)

                case {"type": "get_basic_waypoint_list"}:
                    data = await sync_to_async(get_basic_waypoint_list)()
                    await self.send_json({"type": "get_basic_waypoint_list", "data": data})

                case {"type": "get_auton_waypoint_list"}:
                    data = await sync_to_async(get_auton_waypoint_list)()
                    await self.send_json({"type": "get_auton_waypoint_list", "data": data})

                case {"type": "get_current_basic_course"}:
                    data = await sync_to_async(get_current_basic_course)()
                    await self.send_json({"type": "get_current_basic_course", "data": data})

                case {"type": "get_current_auton_course"}:
                    data = await sync_to_async(get_current_auton_course)()
                    await self.send_json({"type": "get_current_auton_course", "data": data})

                case {"type": "debug", "timestamp": ts}:
                    self.node.get_logger().debug(f"debug message received by consumer, {ts}")

                case _:
                    self.node.get_logger().warning(f"Unhandled message on waypoints: {content}")
        except Exception:
            self.node.get_logger().error(f"Failed to handle message: {content}")
            self.node.get_logger().error(traceback.format_exc())    