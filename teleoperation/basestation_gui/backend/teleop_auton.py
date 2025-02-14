import json
import rclpy
import threading
import traceback
from typing import Any, Type

from channels.generic.websocket import JsonWebsocketConsumer
from rosidl_runtime_py.convert import message_to_ordereddict

from backend.waypoints import (
    get_auton_waypoint_list,
    save_auton_waypoint_list,
)
from mrover.msg import (
    LED,
    StateMachineStateUpdate,
    GPSWaypoint,
    WaypointType,
)
from mrover.srv import EnableAuton
from std_srvs.srv import SetBool

class AutonConsumer(JsonWebsocketConsumer):
    subscribers = []

    def connect(self) -> None:
        self.accept()

        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()

        #Publishers

        #Subscribers
        self.forward_ros_topic("/led", LED, "led")
        self.forward_ros_topic("/nav_state", StateMachineStateUpdate, "nav_state")

        # Services
        self.enable_teleop_srv = self.node.create_client(SetBool, "/enable_teleop")
        self.enable_auton_srv = self.node.create_client(EnableAuton, "/enable_auton")

    def disconnect(self, close_code) -> None:
        for subscriber in self.subscribers:
            self.node.destroy_subscription(subscriber)

        if self.ros_context:
            self.ros_context.shutdown()  # This unblocks rclpy.spin()
        self.ros_node = None  # Clear reference

    def ros_spin(self):
        self.ros_context = rclpy.Context()
        self.ros_context.init()
        self.node = rclpy.create_node("teleop_auton")
        rclpy.spin(self.node)  # This will block until shutdown
        self.node.destroy_node()  # Clean up after spin exits

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

    def send_auton_command(self, waypoints: list[dict], enabled: bool) -> None:
        self.enable_auton_srv.call(
            EnableAuton.Request(
                enable=enabled,
                waypoints=[
                    GPSWaypoint(
                        tag_id=waypoint["tag_id"],
                        latitude_degrees=waypoint["latitude_degrees"],
                        longitude_degrees=waypoint["longitude_degrees"],
                        type=WaypointType(val=int(waypoint["type"])),
                    )
                    for waypoint in waypoints
                ],
            )
        )

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
                case {"type": "auton_enable", "enabled": enabled, "waypoints": waypoints}:
                    self.send_auton_command(waypoints, enabled)
                case {"type": "teleop_enable", "enabled": enabled}:
                    self.enable_teleop_srv.call(SetBool.Request(data=enabled))
                case {
                    "type": "save_auton_waypoint_list",
                    "data": waypoints,
                }:
                    save_auton_waypoint_list(waypoints)
                case {
                    "type": "get_auton_waypoint_list",
                }:
                    self.send_message_as_json({"type": "get_auton_waypoint_list", "data": get_auton_waypoint_list()})
        except:
            self.node.get_logger().error(f"Failed to handle message: {message}")
            self.node.get_logger().error(traceback.format_exc())
