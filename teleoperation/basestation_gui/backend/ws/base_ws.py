import asyncio
import threading
import time
import msgpack
from fastapi import WebSocket
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from backend.managers.ros import get_node

FORWARD_RATE_HZ = 30
FORWARD_INTERVAL_SEC = 1.0 / FORWARD_RATE_HZ


class WebSocketHandler:
    def __init__(self, websocket: WebSocket, endpoint: str):
        self.websocket = websocket
        self.endpoint = endpoint
        self.node = get_node()
        self.subscriptions = []
        self.publishers = []
        self.timers = []
        self.loop = asyncio.get_running_loop()
        self.closed = False
        self.callback_lock = threading.Lock()
        self.last_send_times: dict[str, float] = {}

    async def send_msgpack(self, data):
        if self.closed:
            return
        try:
            packed = msgpack.packb(data, use_bin_type=True)
            await self.websocket.send_bytes(packed)
        except Exception as e:
            print(f"Error sending message on {self.endpoint}: {e}")

    def schedule_send(self, data):
        if self.closed:
            return
        try:
            if self.loop.is_running():
                asyncio.run_coroutine_threadsafe(self.send_msgpack(data), self.loop)
        except RuntimeError:
            pass

    def forward_ros_topic(self, topic_name, topic_type, gui_msg_type):
        self.last_send_times[topic_name] = 0.0

        def callback(ros_message):
            with self.callback_lock:
                if self.closed:
                    return
                now = time.monotonic()
                if now - self.last_send_times[topic_name] < FORWARD_INTERVAL_SEC:
                    return
                self.last_send_times[topic_name] = now
                data_to_send = {"type": gui_msg_type, **message_to_ordereddict(ros_message)}
                self.schedule_send(data_to_send)

        sub = self.node.create_subscription(
            topic_type, topic_name, callback, qos_profile=qos_profile_sensor_data
        )
        self.subscriptions.append(sub)

    async def cleanup(self):
        print(f"Cleaning up {self.endpoint} WebSocket handler...")
        with self.callback_lock:
            self.closed = True

        for sub in self.subscriptions:
            self.node.destroy_subscription(sub)
        self.subscriptions.clear()

        for pub in self.publishers:
            self.node.destroy_publisher(pub)
        self.publishers.clear()

        for timer in self.timers:
            self.node.destroy_timer(timer)
        self.timers.clear()