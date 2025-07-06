# === Standard Library ===
import json
import threading
from typing import Any, Type

# === Third-Party Libraries ===
from channels.generic.websocket import JsonWebsocketConsumer
from rosidl_runtime_py.convert import message_to_ordereddict
from lie import SE3

# === ROS 2 Core Libraries ===
import rclpy
import tf2_ros
from tf2_ros.buffer import Buffer

# === ROS 2 Standard Message Types ===
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Temperature, RelativeHumidity, JointState
from std_msgs.msg import Float32
from std_srvs.srv import SetBool

# === Custom MROVER Message Types ===
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

# === Custom MROVER Services ===
from mrover.srv import EnableAuton, EnableBool, ServoSetPos

# === ROS 2 Node Initialization ===
# For more info: https://docs.ros.org/en/rolling/p/rclpy/
rclpy.init()
node = rclpy.create_node("teleoperation")

def ros_spin():
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

ros_thread = threading.Thread(target=ros_spin, daemon=True)
ros_thread.start()

# === Constants ===
LOCALIZATION_INFO_HZ = 10
heater_names = ["a0", "a1", "b0", "b1"]

class BaseROSConsumer(JsonWebsocketConsumer):
    """
    Base class for WebSocket consumer(s) that interact with ROS 2.

    This consumer sets up a shared ROS 2 node (defined as 'teleoperation'), publishers, service clients,
    and subscriptions to relevant topics. It provides methods to foward ROS topic data to the frontend.

    Attributes:
        node (rclpy.node.Node): Shared ROS node for publishers/subscribers.
        subscribers (List[Subscription]): List of active topic subscriptions.
        timers (List[Timer]): List of periodic timers, e.g., for localization updates.
        buffer (tf2_ros.Buffer): TF buffer for frame transforms.
        tf_listener (tf2_ros.TransformListener): Listener to populate the TF buffer.
    
    Example:
        class GUIConsumer(BaseROSConsumer):
            def receive(self, text_data):
                # handle GUI commands
    """

    subscribers = []
    timers = []

    # if attempting multiple websockets implementation, move any relevant ROS publishers, services, functions, etc. to their
    # respective consumer file
    def connect(self) -> None:
        self.accept()
        self.thr_pub = node.create_publisher(Throttle, "arm_throttle_cmd", 1)
        self.ee_pos_pub = node.create_publisher(IK, "ee_pos_cmd", 1)
        self.ee_vel_pub = node.create_publisher(Twist, "ee_vel_cmd", 1)
        self.joystick_twist_pub = node.create_publisher(Twist, "/joystick_cmd_vel", 1)
        self.controller_twist_pub = node.create_publisher(Twist, "/controller_cmd_vel", 1)
        self.mast_gimbal_pub = node.create_publisher(Throttle, "/mast_gimbal_throttle_cmd", 1)
        self.sa_thr_pub = node.create_publisher(Throttle, "sa_throttle_cmd", 1)

        self.forward_ros_topic("/drive_left_controller_data", ControllerState, "drive_left_state")
        self.forward_ros_topic("/drive_right_controller_data", ControllerState, "drive_right_state")
        self.forward_ros_topic("/led", LED, "led")
        self.forward_ros_topic("/nav_state", StateMachineStateUpdate, "nav_state")
        self.forward_ros_topic("/gps/fix", NavSatFix, "gps_fix")
        self.forward_ros_topic("/science_thermistors", ScienceThermistors, "thermistors")
        self.forward_ros_topic("/science_heater_state", HeaterData, "heater_states")
        self.forward_ros_topic("/science_oxygen_data", Oxygen, "oxygen")
        self.forward_ros_topic("/science_methane_data", Methane, "methane")
        self.forward_ros_topic("/science_uv_data", UV, "uv")
        self.forward_ros_topic("/science_temperature_data", Temperature, "temperature")
        self.forward_ros_topic("/science_humidity_data", RelativeHumidity, "humidity")

        self.forward_ros_topic("/arm_controller_state", ControllerState, "arm_state")
        self.forward_ros_topic("/arm_joint_data", JointState, "fk")
        self.forward_ros_topic("/drive_controller_data", ControllerState, "drive_state")
        self.forward_ros_topic("/sa_controller_state", ControllerState, "sa_state")
        self.forward_ros_topic("/sa_gear_diff_position", Float32, "hexhub_site")
        self.forward_ros_topic("basestation/position", NavSatFix, "basestation_position")
        self.forward_ros_topic("/drone_odometry", NavSatFix, "drone_waypoint")

        # Services
        self.enable_teleop_srv = node.create_client(SetBool, "/enable_teleop")
        self.enable_auton_srv = node.create_client(EnableAuton, "/enable_auton")
        # might need to change service type
        self.gear_diff_set_pos_srv = node.create_client(ServoSetPos, "/sa_gear_diff_set_position")

        # EnableBool Requests
        self.auto_shutoff_service = node.create_client(EnableBool, "/science_change_heater_auto_shutoff_state")
        self.sa_enable_switch_srv = node.create_client(EnableBool, "/sa_enable_limit_switch_sensor_actuator")

        self.heater_services = []
        self.white_leds_services = []
        for name in heater_names:
            self.heater_services.append(node.create_client(EnableBool, "/science_enable_heater_" + name))
        for site in ["a", "b"]:
            self.white_leds_services.append(node.create_client(EnableBool, "/science_enable_white_led_" + site))

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

        self.subscribers.append(node.create_subscription(topic_type, topic_name, callback, 1))


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
            pass
            # node.get_logger().warn(f"Failed to get bearing: {e} Is localization running?")

    def send_auton_command(self, waypoints: list[dict], enabled: bool) -> None:
        self.enable_auton_srv.call(
            EnableAuton.Request(
                enable=enabled,
                waypoints=[
                    GPSWaypoint(
                        tag_id=waypoint["tag_id"],
                        enable_costmap=waypoint["enable_costmap"],
                        latitude_degrees=waypoint["latitude_degrees"],
                        longitude_degrees=waypoint["longitude_degrees"],
                        type=WaypointType(val=int(waypoint["type"])),
                    )
                    for waypoint in waypoints
                ],
            )
        )