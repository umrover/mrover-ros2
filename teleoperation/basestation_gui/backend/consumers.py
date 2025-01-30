import json
import traceback
import rclpy
import tf2_ros
import asyncio
import threading

from typing import Any, Type
from rclpy.service import Service, SrvTypeRequest
from channels.generic.websocket import AsyncWebsocketConsumer
from rosidl_runtime_py.convert import message_to_ordereddict

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

from rclpy.executors import MultiThreadedExecutor

# rclpy.init()
# self.node = rclpy.create_self.node("teleoperation")


# def ros_spin():
#     rclpy.spin(self.node)
#     self.node.destroy_self.node()
#     rclpy.shutdown()


# ros_thread = threading.Thread(target=ros_spin, daemon=True)
# ros_thread.start()

LOCALIZATION_INFO_HZ = 10

cur_mode = "disabled"
heater_names = ["a0", "a1", "b0", "b1"]


class GUIConsumer(AsyncWebsocketConsumer):
    subscribers = []
    timers = []

    async def connect(self) -> None:
        await self.accept()

        # Store Django's event loop for use in ROS callbacks
        self.django_event_loop = asyncio.get_running_loop()

        # Initialize ROS2 inside Django's asyncio event loop
        rclpy.init()
        self.node = rclpy.create_node("teleoperation")

        # Start an AsyncExecutor to handle ROS spin in the background
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)

        # Start ROS spin as an asyncio task
        self.ros_task = asyncio.create_task(self.ros_spin())

        # Example: Creating a timer inside ROS
        self.timers.append(self.node.create_timer(1 / LOCALIZATION_INFO_HZ, self.send_localization_callback))


        # Publishers
        self.thr_pub = self.node.create_publisher(Throttle, "/arm_throttle_cmd", 1)
        self.ee_pos_pub = self.node.create_publisher(IK, "/ee_pos_cmd", 1)
        self.ee_vel_pub = self.node.create_publisher(Vector3, "/ee_vel_cmd", 1)
        self.joystick_twist_pub = self.node.create_publisher(Twist, "/joystick_cmd_vel", 1)
        self.controller_twist_pub = self.node.create_publisher(Twist, "/controller_cmd_vel", 1)
        self.mast_gimbal_pub = self.node.create_publisher(Throttle, "/mast_gimbal_throttle_cmd", 1)

        # Subscribers
        self.forward_ros_topic("/drive_controller_data", ControllerState, "drive_state")
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

        # Services
        self.enable_teleop_srv = self.node.create_client(SetBool, "/enable_teleop")
        self.enable_auton_srv = self.node.create_client(EnableAuton, "/enable_auton")
        self.auto_shutoff_service = self.node.create_client(SetBool, "/science_change_heater_auto_shutoff_state")

        self.heater_services = []
        self.white_leds_services = []
        for name in heater_names:
            self.heater_services.append(self.node.create_client(SetBool, "/science_enable_heater_" + name))
        for site in ["a0", "b0"]:
            self.white_leds_services.append(self.node.create_client(SetBool, "/science_enable_white_led_" + site))

        self.buffer = Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buffer, self.node)


    async def disconnect(self, close_code):
        self.node.get_logger().info("Disconnecting!")
        
        # Destroy all subscriptions and timers
        for subscriber in self.subscribers:
            self.node.destroy_subscription(subscriber)
        for timer in self.timers:
            self.node.destroy_timer(timer)

        # Stop ROS2 executor
        if hasattr(self, "ros_task"):
            self.ros_task.cancel()
        
        # Properly shutdown ROS2
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()

    async def ros_spin(self):
        """Runs ROS2 spin inside Django’s asyncio event loop."""
        try:
            while rclpy.ok():
                self.executor.spin_once(timeout_sec=0.1)  # Process ROS2 events
                await asyncio.sleep(0)  # Let Django’s event loop run
        except asyncio.CancelledError:
            pass  # Allow task cancellation without error

    def forward_ros_topic(self, topic_name: str, topic_type: Type, gui_msg_type: str) -> None:
        """
        Subscribes to a ROS topic and forwards messages to the GUI as JSON

        @param topic_name:      ROS topic name
        @param topic_type:      ROS message type
        @param gui_msg_type:    String to identify the message type in the GUI
        """

        def callback(ros_message: Any):
            if self.scope.get("closed", False):
                return
            # Run the callback asynchronously on the main event loop
            async def send_message():
                await self.send_message_as_json({"type": gui_msg_type, **message_to_ordereddict(ros_message)})

            # Use Django's stored event loop to avoid `get_running_loop()` error
            asyncio.run_coroutine_threadsafe(send_message(), self.django_event_loop)

        self.subscribers.append(self.node.create_subscription(topic_type, topic_name, callback, qos_profile=1))

    async def send_message_as_json(self, msg: dict):
        try:
            await self.send(text_data=json.dumps(msg))
        except Exception as e:
            self.node.get_logger().warning(f"Failed to send message: {e} {msg}")

    def send_localization_callback(self):
        try:
            base_link_in_map = SE3.from_tf_tree(self.buffer, "map", "base_link")

            if hasattr(self, "django_event_loop"):
                self.django_event_loop.call_soon_threadsafe(
                    asyncio.create_task, self.send_localization_callback_async(base_link_in_map)
                )
            else:
                self.node.get_logger().warn("Django event loop is not available!")

        except Exception as e:
            self.node.get_logger().warn(f"Failed to get bearing: {e} Is localization running?")

    # Async function that will send the message
    async def send_localization_callback_async(self, base_link_in_map):
        await self.send_message_as_json(
            {
                "type": "orientation",
                "orientation": base_link_in_map.quat().tolist(),
            }
        )

    def send_auton_command(self, waypoints: list[dict], enabled: bool) -> None:
        req = EnableAuton.Request(
            enable=enabled,
            waypoints=[
                GPSWaypoint(
                    tag_id=int(waypoint["tag_id"]),
                    latitude_degrees=float(waypoint["latitude_degrees"]),
                    longitude_degrees=float(waypoint["longitude_degrees"]),
                    type=WaypointType(val=int(waypoint["type"])),
                )
                for waypoint in waypoints
            ],
        )
        asyncio.create_task(self.call_service_async(self.enable_auton_srv, req, "auton_enable"))

    async def call_service_async(self, service: Service, request: SrvTypeRequest, gui_msg_type: str):
        try:
            # Your code to call the service asynchronously
            self.node.get_logger().info(f"Calling service with request {request}")
            result = await service.call_async(request)
            self.node.get_logger().info(f"Service call result: {result}")
            await self.send_message_as_json({"type": gui_msg_type, "success": result.result().success})
        except Exception as e:
            self.node.get_logger().error(f"Error in call_service_async: {e}")

    async def receive(self, text_data=None, bytes_data=None, **kwargs) -> None:
        """
        Callback function when a message is received in the Websocket

        @param text_data:   Stringfied JSON message
        """

        global cur_mode

        if text_data is None:
            self.node.get_logger().warning("Expecting text but received binary on GUI websocket...")

        try:
            message = json.loads(text_data)
        except json.JSONDecodeError as e:
            self.node.get_logger().warning(f"Failed to decode JSON: {e}")

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
                            await asyncio.to_thread(send_joystick_twist, device_input, self.joystick_twist_pub)
                        case "ra_controller":
                            await asyncio.to_thread(send_controller_twist, device_input, self.controller_twist_pub)
                            await asyncio.to_thread(
                                send_ra_controls,
                                cur_mode,
                                device_input,
                                self.node,
                                self.thr_pub,
                                self.ee_pos_pub,
                                self.ee_vel_pub,
                                self.buffer,
                            )
                        case "mast_keyboard":
                            await asyncio.to_thread(send_mast_controls, device_input, self.mast_gimbal_pub)
                case {
                    "type": "ra_mode",
                    "mode": mode,
                }:
                    cur_mode = mode
                    self.node.get_logger().debug(f"publishing to {cur_mode}")
                case {"type": "auton_enable", "enabled": enabled, "waypoints": waypoints}:
                    self.send_auton_command(waypoints, enabled)
                case {"type": "teleop_enable", "enabled": enabled}:
                    asyncio.create_task(self.call_service_async( self.enable_teleop_srv, SetBool.Request(data=enabled), "teleop_enable"))
                case {
                    "type": "save_auton_waypoint_list",
                    "data": waypoints,
                }:
                    self.node.get_logger().info("SAVING waypoints!")
                    await asyncio.to_thread(save_auton_waypoint_list, waypoints)
                    self.node.get_logger().info("DONE SAVING waypoints!")
                case {
                    "type": "save_basic_waypoint_list",
                    "data": waypoints,
                }:
                    await asyncio.to_thread(save_basic_waypoint_list, waypoints)
                case {
                    "type": "get_basic_waypoint_list",
                }:
                    waypoints = await asyncio.to_thread(get_basic_waypoint_list)
                    await self.send_message_as_json({"type": "get_basic_waypoint_list", "data": waypoints})
                case {
                    "type": "get_auton_waypoint_list",
                }:
                    self.node.get_logger().info("Getting waypoints!")
                    waypoints = await asyncio.to_thread(get_auton_waypoint_list)
                    self.node.get_logger().info("Got waypoints!")
                    await self.send_message_as_json({"type": "get_auton_waypoint_list", "data": waypoints})
                case {
                    "type": "heater_enable",
                    "enabled": enabled, 
                    "heater": heater
                }:
                    req = SetBool.Request(data=enabled)
                    asyncio.create_task(self.call_service_async(self.heater_services[heater_names.index(heater)], req, "heater_enable"))
                case {
                    "type": "auto_shutoff",
                    "shutoff": shutoff
                }:
                    req = SetBool.Request(data=shutoff)
                    asyncio.create_task(self.call_service_async(self.auto_shutoff_service, req, "auto_shutoff"))
                case {
                    "type": "white_leds",
                    "site": site,
                    "enabled": enabled
                }:
                    req = SetBool.Request(data=enabled)
                    asyncio.create_task(self.call_service_async(self.white_leds_services[site], req, "white_leds"))

                case _:
                    self.node.get_logger().warning(f"Unhandled message: {message}")
        except:
            self.node.get_logger().error(f"Failed to handle message: {message}")
            self.node.get_logger().error(traceback.format_exc())
