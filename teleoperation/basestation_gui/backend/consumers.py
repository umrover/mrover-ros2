import json
import traceback
from typing import Any, Type

from channels.generic.websocket import AsyncWebsocketConsumer
import asyncio
from rosidl_runtime_py.convert import message_to_ordereddict

import rclpy

import tf2_ros
from tf2_ros.buffer import Buffer
import numpy as np
from backend.drive_controls import send_joystick_twist, send_controller_twist
from backend.input import DeviceInputs
from backend.models import BasicWaypoint, AutonWaypoint
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import NavSatFix, Temperature, RelativeHumidity
from lie import SE3
from mrover.msg import Throttle, IK, ControllerState, HeaterData, ScienceThermistors, Oxygen, Methane, UV
from backend.ra_controls import send_ra_controls
from backend.mast_controls import send_mast_controls
from std_srvs.srv import SetBool

import threading

import logging
logger = logging.getLogger('django')

rclpy.init()
node = rclpy.create_node('teleoperation')
def ros_spin():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    rclpy.spin(node)

ros_thread = threading.Thread(target=ros_spin, daemon=True)
ros_thread.start()

cur_mode = "disabled"
heater_names = ['a0', 'a1', 'b0', 'b1']

class GUIConsumer(AsyncWebsocketConsumer):
    subscribers = []
    timers = []
    
    async def connect(self) -> None:
        await self.accept()
        # Publishers
        self.thr_pub = node.create_publisher(Throttle, "/arm_throttle_cmd",1)
        self.ee_pos_pub = node.create_publisher(IK, "/ee_pos_cmd",1)
        self.ee_vel_pub = node.create_publisher(Vector3, "/ee_vel_cmd",1)
        self.joystick_twist_pub = node.create_publisher(Twist, "/joystick_cmd_vel", 1)
        self.controller_twist_pub = node.create_publisher(Twist, "/controller_cmd_vel", 1)
        self.mast_gimbal_pub = node.create_publisher(Throttle, "/mast_gimbal_throttle_cmd", 1)

        # Subscribers
        self.forward_ros_topic("/drive_controller_data", ControllerState, "drive_state")
        self.forward_ros_topic("/science_thermistors", ScienceThermistors, "thermistors")
        self.forward_ros_topic("/science_heater_state", HeaterData, "heater_states")
        self.forward_ros_topic("/science_oxygen_data", Oxygen, "oxygen")
        self.forward_ros_topic("/science_methane_data", Methane, "methane")
        self.forward_ros_topic("/science_uv_data", UV, "uv")
        self.forward_ros_topic("/science_temperature_data", Temperature, "temperature")
        self.forward_ros_topic("/science_humidity_data", RelativeHumidity, "humidity")

        # Services
        self.auto_shutoff_service = node.create_client(SetBool, "/science_change_heater_auto_shutoff_state")

        self.heater_services = []
        self.white_leds_services = []
        for name in heater_names:
            self.heater_services.append(node.create_client(SetBool, "/science_enable_heater_" + name))
        for site in ['a0', 'b0']:
            self.white_leds_services.append(node.create_client(SetBool, "/science_enable_white_led_" + site))

        self.buffer = Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buffer, node)

    async def disconnect(self, close_code):
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
            # Run the callback asynchronously on the main event loop
            async def send_message():
                await self.send_message_as_json({"type": gui_msg_type, **message_to_ordereddict(ros_message)})

            loop = asyncio.get_event_loop()
            if loop.is_running():
                # If the loop is running, use `run_coroutine_threadsafe` to schedule the task
                asyncio.run_coroutine_threadsafe(send_message(), loop)
            else:
                # If the loop isn't running (which is rare for a running asyncio app), we create and run a new loop
                new_loop = asyncio.new_event_loop()
                asyncio.set_event_loop(new_loop)
                new_loop.run_until_complete(send_message())
        self.subscribers.append(node.create_subscription(topic_type, topic_name , callback, qos_profile=1))

    async def send_message_as_json(self, msg: dict):
        try:
            await self.send(text_data=json.dumps(msg))
        except Exception as e:
            node.get_logger().warning(f"Failed to send message: {e}")


    async def receive(self, text_data=None, bytes_data=None, **kwargs) -> None:
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
                #sending controls
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
                            await asyncio.to_thread(send_controller_twist,device_input, self.controller_twist_pub)
                            await asyncio.to_thread(send_ra_controls,cur_mode,device_input,node, self.thr_pub, self.ee_pos_pub, self.ee_vel_pub, self.buffer)
                        case "mast_keyboard":
                            await asyncio.to_thread(send_mast_controls,device_input, self.mast_gimbal_pub)
                case{
                    "type":"ra_mode",
                    "mode": mode,
                }:
                    cur_mode = mode
                    node.get_logger().debug(f"publishing to {cur_mode}")
                case{
                    "type": "save_auton_waypoint_list",
                    "data": waypoints,
                }:
                    AutonWaypoint.objects.all().delete()
                    AutonWaypoint.objects.bulk_create(
                    [
                            AutonWaypoint(
                                tag_id=w["id"],
                                type=w["type"],
                                latitude=w["lat"],
                                longitude=w["lon"],
                                name=w["name"],
                            )
                            for w in waypoints
                        ]
                    )
                case{
                    "type": "save_basic_waypoint_list",
                    "data": waypoints,
                }:
                    BasicWaypoint.objects.all().delete()
                    # when adding a new waypoint to the db, primary key is auto generated (see models.py)
                    BasicWaypoint.objects.bulk_create(
                        [BasicWaypoint(drone=w["drone"], latitude=w["lat"], longitude=w["lon"], name=w["name"]) for w in waypoints]
                    )
                case{
                    "type": "get_basic_waypoint_list",
                }:
                    await self.send_message_as_json(
                        {
                            "type": "get_basic_waypoint_list",
                            "data": [
                                {"name": w.name, "drone": w.drone, "lat": w.latitude, "lon": w.longitude}
                                for w in BasicWaypoint.objects.all()
                            ],
                        }
                    )

                case{
                    "type": "heater_enable",
                    "enabled": enabled, 
                    "heater": heater
                }:
                    self.heater_services[heater_names.index(heater)].call_async(SetBool.Request(data=enabled))

                case{
                    "type": "auto_shutoff",
                    "shutoff": shutoff
                }:
                    self.auto_shutoff_service.call_async(SetBool.Request(data=shutoff))
                case{
                    "type": "white_leds",
                    "site": site,
                    "enabled": enabled
                }:
                    self.white_leds_services[site].call_async(SetBool.Request(data=enabled))
                    
                case _:
                    node.get_logger().warning(f"Unhandled message: {message}")
        except:
            node.get_logger().error(f"Failed to handle message: {message}")
            node.get_logger().error(traceback.format_exc())