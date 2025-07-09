import json
import rclpy
import threading
import traceback
from typing import Any, Type

from channels.generic.websocket import JsonWebsocketConsumer
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import (
    Temperature,
    RelativeHumidity
)

from mrover.msg import (
    ScienceThermistors,
    HeaterData,
    Oxygen,
    Methane,
    UV,
)

from std_srvs.srv import SetBool

heater_names = ["a0", "a1", "b0", "b1"]

class ISHConsumer(JsonWebsocketConsumer):
    subscribers = []

    def connect(self) -> None:
        self.accept()

        # Initialize ROS node **before** spinning
        self.ros_context = rclpy.Context()
        self.ros_context.init()
        self.node = rclpy.create_node("teleop_ish")

        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()

        # Publishers

        # Subscribers
        self.forward_ros_topic("/science_thermistors", ScienceThermistors, "thermistors")
        self.forward_ros_topic("/science_heater_state", HeaterData, "heater_states")
        self.forward_ros_topic("/science_oxygen_data", Oxygen, "oxygen")
        self.forward_ros_topic("/science_methane_data", Methane, "methane")
        self.forward_ros_topic("/science_uv_data", UV, "uv")
        self.forward_ros_topic("/science_temperature_data", Temperature, "temperature")
        self.forward_ros_topic("/science_humidity_data", RelativeHumidity, "humidity")

        # Services
        self.auto_shutoff_service = self.node.create_client(SetBool, "/science_change_heater_auto_shutoff_state")

        self.heater_services = []
        self.white_leds_services = []
        for name in heater_names:
            self.heater_services.append(self.node.create_client(SetBool, "/science_enable_heater_" + name))
        for site in ["a0", "b0"]:
            self.white_leds_services.append(self.node.create_client(SetBool, "/science_enable_white_led_" + site))

    def disconnect(self, close_code) -> None:
        for subscriber in self.subscribers:
            self.node.destroy_subscription(subscriber)

        if self.ros_context:
            self.ros_context.shutdown()  # This unblocks rclpy.spin()
        self.node.destroy_node()  # Clean up node
        self.node = None  # Clear reference

    def ros_spin(self):
        executor = MultiThreadedExecutor()
        executor.add_node(self.node)

        try:
            executor.spin()  # Allows multiple threads to handle ROS callbacks safely
        except Exception as e:
            print(f"Exception in ROS spin: {e}")
        finally:
            self.node.destroy_node()

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
                case {"type": "heater_enable", "enabled": enabled, "heater": heater}:
                    self.heater_services[heater_names.index(heater)].call(SetBool.Request(data=enabled))

                case {"type": "auto_shutoff", "shutoff": shutoff}:
                    self.auto_shutoff_service.call(SetBool.Request(data=shutoff))
                    
                case {"type": "white_leds", "site": site, "enabled": enabled}:
                    self.white_leds_services[site].call(SetBool.Request(data=enabled))

        except:
            self.node.get_logger().error(f"Failed to handle message: {message}")
            self.node.get_logger().error(traceback.format_exc())
