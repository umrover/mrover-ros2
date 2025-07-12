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

cur_ra_mode: str = "disabled"
cur_sa_mode: str = "disabled"

class ArmConsumer(JsonWebsocketConsumer):
    subscribers = []
    timers = []

    def connect(self) -> None:
        self.accept()

        self.node = get_node()
        self.ros_context = get_context()

        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()

        print("arm consumer started")

        # Topic Publishers
        self.thr_pub = self.node.create_publisher(Throttle, "arm_throttle_cmd", 1)
        self.ee_pos_pub = self.node.create_publisher(IK, "ee_pos_cmd", 1)
        self.ee_vel_pub = self.node.create_publisher(Twist, "ee_vel_cmd", 1)
        self.controller_twist_pub = self.node.create_publisher(Twist, "/controller_cmd_vel", 1)
        self.sa_thr_pub = self.node.create_publisher(Throttle, "sa_throttle_cmd", 1)

        # Forwards ROS topic to GUI
        self.forward_ros_topic("/arm_controller_state", ControllerState, "arm_state")
        self.forward_ros_topic("/sa_controller_state", ControllerState, "sa_state")
        self.forward_ros_topic("/arm_joint_data", JointState, "fk")

        # Services

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

        global cur_ra_mode
        global cur_sa_mode

        if text_data is None:
            self.node.get_logger().warning("Expecting text but received binary on GUI websocket...")

        try:
            message = json.loads(text_data)
        except json.JSONDecodeError as e:
            self.node.get_logger().warning(f"Failed to decode JSON: {e}")

        try:
            match message:
                case {
                    "type": "ra_controller",
                    "axes": axes,
                    "buttons": buttons,
                }:
                    device_input = DeviceInputs(axes, buttons)
                    send_controller_twist(device_input, self.controller_twist_pub)
                    send_ra_controls(
                        cur_ra_mode,
                        device_input,
                        self.node,
                        self.thr_pub,
                        self.ee_pos_pub,
                        self.ee_vel_pub,
                        self.buffer,
                    )
                            
                case {
                    "type": "ra_mode",
                    "mode": ra_mode,
                }:
                    cur_ra_mode = ra_mode

                case {
                    "type": "sa_controller",
                    "axes": axes,
                    "buttons": buttons,
                    "site": site
                }:
                    device_input = DeviceInputs(axes, buttons)
                    if(site == 0):
                        send_sa_controls(cur_sa_mode, 0, device_input, self.sa_thr_pub)
                    elif(site == 1):
                        send_sa_controls(cur_sa_mode, 1, device_input, self.sa_thr_pub)
                    else:
                        self.node.get_logger().warning(f"Unhandled Site: {site}")

                case {
                    "type": "sa_mode",
                    "mode": sa_mode,
                }:
                    cur_sa_mode = sa_mode

                case _:
                    self.node.get_logger().warning(f"Unhandled message on arm: {message}")
        except:
            self.node.get_logger().error(f"Failed to handle message: {message}")
            self.node.get_logger().error(traceback.format_exc())