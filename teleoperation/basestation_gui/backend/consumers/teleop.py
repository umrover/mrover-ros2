import json
import traceback
from typing import Any, Type

from channels.generic.websocket import JsonWebsocketConsumer
from rosidl_runtime_py.convert import message_to_ordereddict

import rclpy
import tf2_ros
import asyncio
import threading

from tf2_ros.buffer import Buffer
from lie import SE3
from backend.drive_controls import send_joystick_twist, send_controller_twist
from backend.input import DeviceInputs
from backend.ra_controls import send_ra_controls
from backend.mast_controls import send_mast_controls
from backend.waypoints import (
    get_auton_waypoint_list,
    get_basic_waypoint_list,
    save_auton_waypoint_list,
    save_basic_waypoint_list,
)
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import NavSatFix, Temperature, RelativeHumidity
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
from mrover.srv import EnableAuton
from std_srvs.srv import SetBool

rclpy.init()
node = rclpy.create_node("teleoperation")


def ros_spin():
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


ros_thread = threading.Thread(target=ros_spin, daemon=True)
ros_thread.start()

LOCALIZATION_INFO_HZ = 10

cur_mode = "disabled"
heater_names = ["a0", "a1", "b0", "b1"]


class GeneralConsumer(JsonWebsocketConsumer):
    subscribers = []
    timers = []

    def connect(self) -> None:
        self.accept()

        # Publishers
        self.thr_pub = node.create_publisher(Throttle, "arm_throttle_cmd", 1)
        self.ee_pos_pub = node.create_publisher(IK, "ee_pos_cmd", 1)
        self.ee_vel_pub = node.create_publisher(Vector3, "ee_vel_cmd", 1)
        self.joystick_twist_pub = node.create_publisher(Twist, "/joystick_cmd_vel", 1)
        self.controller_twist_pub = node.create_publisher(Twist, "/controller_cmd_vel", 1)
        self.mast_gimbal_pub = node.create_publisher(Throttle, "/mast_gimbal_throttle_cmd", 1)
        self.sa_thr_pub = node.create_publisher(Throttle, "sa_throttle_cmd", 1) # new

        # Subscribers
        self.forward_ros_topic("/drive_left_controller_data", ControllerState, "drive_left_state")
        self.forward_ros_topic("/drive_right_controller_data", ControllerState, "drive_right_state")
        self.forward_ros_topic("/gps/fix", NavSatFix, "gps_fix")

        # Services

        self.buffer = Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buffer, node)

        self.timers.append(node.create_timer(1 / LOCALIZATION_INFO_HZ, self.send_localization_callback))

    def disconnect(self, close_code) -> None:
        for subscriber in self.subscribers:
            node.destroy_subscription(subscriber)
        for timer in self.timers:
            node.destroy_timer(timer)

    def forward_ros_topic(self, topic_name: str, topic_type: Type, gui_msg_type: str) -> None:
        """
        Subscribes to a ROS topic and forwards messages to the GUI as JSON

        @param topic_name:      ROS topic name
        @param topic_type:      ROS message type
        @param gui_msg_type:    String to identify the message type in the GUI
        """

        def callback(ros_message: Any):
            # Formatting a ROS message as a string outputs YAML
            # Parse it back into a dictionary, so we can send it as JSON
            self.send_message_as_json({"type": gui_msg_type, **message_to_ordereddict(ros_message)})

        self.subscribers.append(node.create_subscription(topic_type, topic_name, callback, qos_profile=1))

    def send_message_as_json(self, msg: dict):
        try:
            self.send(text_data=json.dumps(msg))
        except Exception as e:
            node.get_logger().warning(f"Failed to send message: {e}")

    def send_localization_callback(self):
        try:
            base_link_in_map = SE3.from_tf_tree(self.buffer, "map", "base_link")
            self.send_message_as_json(
                {
                    "type": "orientation",
                    "orientation": base_link_in_map.quat().tolist(),
                }
            )
        except Exception as e:
            node.get_logger().warn(f"Failed to get bearing: {e} Is localization running?")

    def receive(self, text_data=None, bytes_data=None, **kwargs) -> None:
        """
        Callback function when a message is received in the Websocket

        @param text_data:   Stringfied JSON message
        """

        global cur_mode

        if text_data is None:
            node.get_logger().warning("Expecting text but received binary on GUI websocket...")

        try:
            message = json.loads(text_data)
        except json.JSONDecodeError as e:
            node.get_logger().warning(f"Failed to decode JSON: {e}")

        try:
            match message:
                # sending controls
                case {
                    "type": "joystick" | "mast_keyboard" | "ra_controller",
                    "axes": axes,
                    "buttons": buttons,
                }:
                    device_input = DeviceInputs(axes, buttons)
                    match message["type"]:
                        case "joystick":
                            send_joystick_twist(device_input, self.joystick_twist_pub)
                        case "ra_controller":
                            send_controller_twist(device_input, self.controller_twist_pub)
                            send_ra_controls(
                                cur_mode,
                                device_input,
                                node,
                                self.thr_pub,
                                self.ee_pos_pub,
                                self.ee_vel_pub,
                                self.buffer,
                            )
                        case "mast_keyboard":
                            send_mast_controls(device_input, self.mast_gimbal_pub)
                case {
                    "type": "ra_mode",
                    "mode": mode,
                }:
                    cur_mode = mode
                    node.get_logger().debug(f"publishing to {cur_mode}")
                case {
                    "type": "save_basic_waypoint_list",
                    "data": waypoints,
                }:
                    save_basic_waypoint_list(waypoints)
                case {
                    "type": "get_basic_waypoint_list",
                }:
                    self.send_message_as_json({"type": "get_basic_waypoint_list", "data": get_basic_waypoint_list()})
                case _:
                    node.get_logger().warning(f"Unhandled message: {message}")
        except:
            node.get_logger().error(f"Failed to handle message: {message}")
            node.get_logger().error(traceback.format_exc())
