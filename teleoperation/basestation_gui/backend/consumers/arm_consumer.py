import json
import traceback
from typing import Any, Type
import asyncio

# CONVERTED: Use the async version of the consumer
from channels.generic.websocket import AsyncJsonWebsocketConsumer
from asgiref.sync import sync_to_async
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.qos import qos_profile_sensor_data

# Use the centralized ROS manager
from backend.consumers.ros_manager import get_node

from backend.drive_controls import send_controller_twist
from backend.input import DeviceInputs
from backend.ra_controls import send_ra_controls
from backend.sa_controls import send_sa_controls
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from mrover.msg import (
    Throttle,
    IK,
    ControllerState,
)

class ArmConsumer(AsyncJsonWebsocketConsumer):
    # These are now instance variables, not class variables
    # subscribers: list
    # timers: list

    async def connect(self) -> None:
        await self.accept()

        self.node = get_node()

        # FIX: Use instance variables instead of class or global variables
        self.subscribers = []
        self.timers = []
        self.cur_ra_mode: str = "disabled"
        self.cur_sa_mode: str = "disabled"
        self.buffer = {} # Assuming this was intended to be an instance buffer

        # Topic Publishers are created once per connection
        self.thr_pub = self.node.create_publisher(Throttle, "arm_throttle_cmd", 1)
        self.ee_pos_pub = self.node.create_publisher(IK, "ee_pos_cmd", 1)
        self.ee_vel_pub = self.node.create_publisher(Twist, "ee_vel_cmd", 1)
        self.controller_twist_pub = self.node.create_publisher(Twist, "/controller_cmd_vel", 1)
        self.sa_thr_pub = self.node.create_publisher(Throttle, "sa_throttle_cmd", 1)

        # Forwards ROS topic to GUI
        await self.forward_ros_topic("/arm_controller_state", ControllerState, "arm_state")
        await self.forward_ros_topic("/sa_controller_state", ControllerState, "sa_state")
        await self.forward_ros_topic("/arm_joint_data", JointState, "fk")

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

        # CONVERTED: Use sync_to_async for blocking rclpy calls
        sub = await sync_to_async(self.node.create_subscription)(
            topic_type, topic_name, callback, qos_profile=qos_profile_sensor_data
        )
        self.subscribers.append(sub)


    async def receive_json(self, content: dict, **kwargs) -> None:
        # CONVERTED: Use `receive_json` which automatically decodes the JSON content
        try:
            match content:
                case {
                    "type": "ra_controller",
                    "axes": axes,
                    "buttons": buttons,
                }:
                    device_input = DeviceInputs(axes, buttons)
                    send_controller_twist(device_input, self.controller_twist_pub)
                    send_ra_controls(
                        self.cur_ra_mode, # FIX: Use instance variable
                        device_input,
                        self.node,
                        self.thr_pub,
                        self.ee_pos_pub,
                        self.ee_vel_pub,
                        self.buffer,
                    )

                case { "type": "ra_mode", "mode": ra_mode }:
                    self.cur_ra_mode = ra_mode # FIX: Use instance variable

                case {
                    "type": "sa_controller",
                    "axes": axes,
                    "buttons": buttons,
                    "site": site
                }:
                    device_input = DeviceInputs(axes, buttons)
                    # FIX: Use instance variable for mode
                    if site == 0:
                        send_sa_controls(self.cur_sa_mode, 0, device_input, self.sa_thr_pub)
                    elif site == 1:
                        send_sa_controls(self.cur_sa_mode, 1, device_input, self.sa_thr_pub)
                    else:
                        self.node.get_logger().warning(f"Unhandled Site: {site}")

                case { "type": "sa_mode", "mode": sa_mode }:
                    self.cur_sa_mode = sa_mode # FIX: Use instance variable

                case _:
                    self.node.get_logger().warning(f"Unhandled message on arm: {content}")
        except Exception:
            self.node.get_logger().error(f"Failed to handle message: {content}")
            self.node.get_logger().error(traceback.format_exc())