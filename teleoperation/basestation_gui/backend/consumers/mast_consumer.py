import json
import traceback
from typing import Any, Type

from channels.generic.websocket import JsonWebsocketConsumer
from rosidl_runtime_py.convert import message_to_ordereddict

import threading
import numpy as np
import cv2
from time import sleep
from rclpy.executors import MultiThreadedExecutor
from backend.consumers.init_node import get_node, get_context

from backend.input import DeviceInputs

from backend.mast_controls import send_mast_controls
from sensor_msgs.msg import Image
from mrover.msg import Throttle
from mrover.srv import (
    PanoramaStart,
    PanoramaEnd,
)

LOCALIZATION_INFO_HZ = 10

class MastConsumer(JsonWebsocketConsumer):
    subscribers = []
    timers = []

    def connect(self) -> None:
        self.accept()

        self.node = get_node()
        self.ros_context = get_context()

        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()

        # Topic Publishers
        self.mast_gimbal_pub = self.node.create_publisher(Throttle, "/mast_gimbal_throttle_cmd", 1)

        # Forwards ROS topic to GUI

        # Services
        self.pano_start_srv = self.node.create_client(PanoramaStart, "/panorama/start")
        self.pano_end_srv = self.node.create_client(PanoramaEnd, "/panorama/end")

    def disconnect(self, close_code) -> None:
        try:
            for subscriber in self.subscribers:
                self.node.destroy_subscription(subscriber)
            self.subscribers.clear()
            self.timers.clear()

            if self.ros_thread.is_alive():
                self.ros_thread.join(timeout=1)
        except Exception as e:
            print(f"Exception during disconnect cleanup: {e}")

    def ros_spin(self) -> None:
        executor = MultiThreadedExecutor(context=self.ros_context)
        executor.add_node(self.node)
        try:
            executor.spin()
        except Exception as e:
            print(f"Exception in ROS spin: {e}")

    def forward_ros_topic(self, topic_name: str, topic_type: Type, gui_msg_type: str) -> None:
        """
        Subscribes to a ROS topic and forwards messages to the GUI as JSON

        @param topic_name:      ROS topic name
        @param topic_type:      ROS message type
        @param gui_msg_type:    String to identify the message type in the GUI
        """

        def callback(ros_message: Any):
            self.send_message_as_json({"type": gui_msg_type, **message_to_ordereddict(ros_message)})

        self.subscribers.append(self.node.create_subscription(topic_type, topic_name, callback, qos_profile=1))


    def start_stop_pano(self, action: str) -> None:
        if action == "start":
            self.node.get_logger().info("Pano start")
            self.pano_start_srv.call_async(PanoramaStart.Request())
        else:
            self.node.get_logger().info("Pano stop")
            self.future = self.pano_end_srv.call_async(PanoramaEnd.Request())
            
            self.future.add_done_callback(self.handle_pano_response)

            self.node.get_logger().warn("Waiting for pano result...")

    def handle_pano_response(self, future):
        self.node.get_logger().warn("Callback run...")

        if future.done():
            result = future.result()
            
            if result is not None and result.success:
                img_msg: Image = result.img
                img_np = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                    img_msg.height, img_msg.width, 4
                )
                timestamp = f"{self.node.get_clock().now().seconds_nanoseconds()[0]}_{self.node.get_clock().now().seconds_nanoseconds()[1]}"
                cv2.imwrite(f"../../data/{timestamp}panorama.png", img_np)
                self.node.get_logger().info(f"Image saved to {timestamp}panorama.png")
            else:
                self.node.get_logger().info("No response...")
        else:
            self.node.get_logger().error("Service call failed...")

    def send_message_as_json(self, msg: dict):
        try:
            self.send(text_data=json.dumps(msg))
        except Exception as e:
            self.node.get_logger().warning(f"Failed to send message: {e}")

    def receive(self, text_data=None, bytes_data=None, **kwargs) -> None:
        """
        Callback function when a message is received in the Websocket

        @param text_data:   Stringfied JSON message
        """

        if text_data is None:
            self.node.get_logger().warning("Expecting text but received binary on GUI websocket...")

        try:
            message = json.loads(text_data)
        except json.JSONDecodeError as e:
            self.node.get_logger().warning(f"Failed to decode JSON: {e}")

        try:
            match message:
                case {
                    "type": "mast_keyboard",
                    "axes": axes,
                    "buttons": buttons,
                }:
                    device_input = DeviceInputs(axes, buttons)
                    send_mast_controls(device_input, self.mast_gimbal_pub)
                case {"type": "pano", "action": a}:
                    self.start_stop_pano(a)
                    
                case _:
                    self.node.get_logger().warning(f"Unhandled message on mast: {message}")
        except:
            self.node.get_logger().error(f"Failed to handle message: {message}")
            self.node.get_logger().error(traceback.format_exc())