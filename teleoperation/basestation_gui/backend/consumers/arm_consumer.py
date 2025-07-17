import json
import traceback
from typing import Any, Type
import asyncio

from channels.generic.websocket import AsyncJsonWebsocketConsumer
from asgiref.sync import sync_to_async

from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from backend.consumers.init_node import get_node, get_context
from tf2_ros.buffer import Buffer
import tf2_ros
import rclpy

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
    
    async def connect(self) -> None:
        await self.accept()

        self.node = get_node()
        self.ros_context = get_context()

        self.cur_ra_mode: str = "disabled"
        self.cur_sa_mode: str = "disabled"
        self.subscribers = []

        # Topic Publishers
        self.thr_pub = self.node.create_publisher(Throttle, "arm_throttle_cmd", 1)
        self.ee_pos_pub = self.node.create_publisher(IK, "ee_pos_cmd", 1)
        self.ee_vel_pub = self.node.create_publisher(Twist, "ee_vel_cmd", 1)
        self.controller_twist_pub = self.node.create_publisher(Twist, "/controller_cmd_vel", 1)
        self.sa_thr_pub = self.node.create_publisher(Throttle, "sa_throttle_cmd", 1)

        # Forwards ROS topic to GUI
        await self.forward_ros_topic("/arm_controller_state", ControllerState, "arm_state")
        await self.forward_ros_topic("/sa_controller_state", ControllerState, "sa_state")
        await self.forward_ros_topic("/arm_joint_data", JointState, "joint_data")

        # Buffer
        # Running the listener creation in an executor to be safe if it blocks
        self.buffer = Buffer()
        self.tf_listener = await asyncio.to_thread(tf2_ros.TransformListener, self.buffer, self.node)

        self.ros_task = asyncio.to_thread(self.ros_spin)

    async def disconnect(self, close_code) -> None:
        print("ArmConsumer disconnecting...")
        # 4. Implement a graceful shutdown for ROS
        try:
            # First, destroy ROS entities
            for subscriber in self.subscribers:
                # Use sync_to_async to run the synchronous destroy method
                await sync_to_async(self.node.destroy_subscription)(subscriber)
            self.subscribers.clear()
            
            # Then, shut down the rclpy context to unblock executor.spin()
            if self.ros_context.is_valid():
                rclpy.shutdown(context=self.ros_context)
            
            # The self.ros_task will now complete or raise an exception
            await self.ros_task
            print("ROS spin task finished.")
            
        except Exception as e:
            print(f"Exception during disconnect cleanup: {e}")
            
    def ros_spin(self) -> None:
        """This function is blocking and will be run in a separate thread."""
        executor = MultiThreadedExecutor(context=self.ros_context)
        executor.add_node(self.node)
        try:
            # This will block until rclpy.shutdown() is called on its context
            executor.spin()
        except (ExternalShutdownException, RuntimeError):
            # This is the expected exception when rclpy.shutdown() is called
            print("ROS executor has been shut down.")
        finally:
            print("ROS spin loop exited.")

    async def forward_ros_topic(self, topic_name: str, topic_type: Type, gui_msg_type: str) -> None:
        """Subscribes to a ROS topic and forwards messages to the GUI."""
        
        # 6. Safely call async `send_json` from a synchronous ROS thread
        # The ROS callback runs in a different thread. We need a thread-safe
        # way to call our async send function on the main event loop.
        loop = asyncio.get_running_loop()

        def callback(ros_message: Any):
            data_to_send = {"type": gui_msg_type, **message_to_ordereddict(ros_message)}
            # Schedule the coroutine to run on the main event loop
            asyncio.run_coroutine_threadsafe(self.send_json(data_to_send), loop)

        sub = await sync_to_async(self.node.create_subscription)(
            topic_type, topic_name, callback, qos_profile=1
        )
        self.subscribers.append(sub)

    async def receive_json(self, content, **kwargs) -> None:
        """Callback function for received JSON messages."""
        message = content
        try:
            match message:
                case { "type": "ra_controller", "axes": axes, "buttons": buttons }:
                    device_input = DeviceInputs(axes, buttons)
                    # Run blocking ROS calls in a thread to not block the event loop
                    await asyncio.to_thread(send_controller_twist, device_input, self.controller_twist_pub)
                    await asyncio.to_thread(send_ra_controls, self.cur_ra_mode, device_input, self.node, self.thr_pub, self.ee_pos_pub, self.ee_vel_pub, self.buffer)
                            
                case { "type": "ra_mode", "mode": ra_mode }:
                    self.cur_ra_mode = ra_mode

                case { "type": "sa_controller", "axes": axes, "buttons": buttons, "site": site }:
                    device_input = DeviceInputs(axes, buttons)
                    await asyncio.to_thread(send_sa_controls, self.cur_sa_mode, site, device_input, self.sa_thr_pub)

                case { "type": "sa_mode", "mode": sa_mode }:
                    self.cur_sa_mode = sa_mode

                case _:
                    self.node.get_logger().warning(f"Unhandled message on arm: {message}")
        except Exception:
            self.node.get_logger().error(f"Failed to handle message: {message}")
            self.node.get_logger().error(traceback.format_exc())