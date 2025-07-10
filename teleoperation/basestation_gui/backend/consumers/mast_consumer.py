import json
import traceback
from typing import Any, Type

from channels.generic.websocket import JsonWebsocketConsumer
from numpy import float32
from rosidl_runtime_py.convert import message_to_ordereddict

import rclpy
import tf2_ros
import asyncio
import threading
from rclpy.executors import MultiThreadedExecutor
from backend.consumers.init_node import get_node, get_context

from tf2_ros.buffer import Buffer
from lie import SE3
from backend.drive_controls import send_joystick_twist, send_controller_twist
from backend.input import DeviceInputs
from backend.ra_controls import send_ra_controls
from backend.sa_controls import send_sa_controls
from backend.mast_controls import send_mast_controls
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
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import NavSatFix, Temperature, RelativeHumidity, JointState
from mrover.msg import (
    Throttle,
    IK,
    ControllerState,
    LED,
    StateMachineStateUpdate,
    GPSWaypoint,
    WaypointType,
    HeaterData,
    ScienceThermistors,
    Oxygen,
    Methane,
    UV,
)
from mrover.srv import (
    EnableAuton, 
    EnableBool,
    ServoSetPos 
)
from std_srvs.srv import SetBool
from std_msgs.msg import Float32

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

        print("mast consumer started")

        # Topic Publishers
        self.mast_gimbal_pub = self.node.create_publisher(Throttle, "/mast_gimbal_throttle_cmd", 1)

        # Forwards ROS topic to GUI

        # Services

    def disconnect(self, close_code) -> None:
        for subscriber in self.subscribers:
            self.node.destroy_subscription(subscriber)
        self.subscribers.clear()
        self.timers.clear()

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

                case _:
                    self.node.get_logger().warning(f"Unhandled message on mast: {message}")
        except:
            self.node.get_logger().error(f"Failed to handle message: {message}")
            self.node.get_logger().error(traceback.format_exc())