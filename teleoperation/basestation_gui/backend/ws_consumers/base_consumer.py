import asyncio
import msgpack
from fastapi import WebSocket
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from backend.ros_manager import get_node

class WebSocketHandler:
    def __init__(self, websocket: WebSocket, endpoint: str):
        self.websocket = websocket
        self.endpoint = endpoint
        self.node = get_node()
        self.subscribers = []
        self.timers = []
        self.loop = asyncio.get_running_loop()

    async def send_msgpack(self, data):
        """Send msgpack encoded data to the WebSocket"""
        try:
            packed = msgpack.packb(data, use_bin_type=True)
            await self.websocket.send_bytes(packed)
        except Exception as e:
            print(f"Error sending message on {self.endpoint}: {e}")

    def forward_ros_topic(self, topic_name, topic_type, gui_msg_type):
        """Subscribe to a ROS topic and forward messages to WebSocket"""
        def callback(ros_message):
            data_to_send = {"type": gui_msg_type, **message_to_ordereddict(ros_message)}
            asyncio.run_coroutine_threadsafe(self.send_msgpack(data_to_send), self.loop)

        sub = self.node.create_subscription(
            topic_type, topic_name, callback, qos_profile=qos_profile_sensor_data
        )
        self.subscribers.append(sub)

    async def cleanup(self):
        """Cleanup ROS resources"""
        print(f"Cleaning up {self.endpoint} WebSocket handler...")
        for sub in self.subscribers:
            self.node.destroy_subscription(sub)
        self.subscribers.clear()

        for timer in self.timers:
            self.node.destroy_timer(timer)
        self.timers.clear()