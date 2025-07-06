import json
import traceback
from .base_consumer import BaseROSConsumer

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
from mrover.srv import (
    EnableAuton, 
    EnableBool,
    ServoSetPos 
)
from std_srvs.srv import SetBool

# rclpy.init()
# node = rclpy.create_node("teleoperation")


# def ros_spin():
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# ros_thread = threading.Thread(target=ros_spin, daemon=True)
# ros_thread.start()

LOCALIZATION_INFO_HZ = 10

cur_mode = "disabled"
cur_sa_mode = "disabled"
heater_names = ["a0", "a1", "b0", "b1"]


class GUIConsumer(BaseROSConsumer):
    def receive(self, text_data=None, bytes_data=None, **kwargs) -> None:
        """
        Callback function when a message is received in the Websocket

        @param text_data:   Stringfied JSON message
        """

        global cur_mode
        global cur_sa_mode
        global node

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
                case{
                    "type": "sa_controller",
                    "axes": axes,
                    "buttons": buttons,
                    "site": site
                }:
                    device_input = DeviceInputs(axes, buttons)
                    if(site == 0):
                        # node.get_logger().info("here", site)
                        send_sa_controls(cur_sa_mode, 0, device_input, self.sa_thr_pub)
                    elif(site == 1):
                        # node.get_logger().info("here1", site)
                        send_sa_controls(cur_sa_mode, 1, device_input, self.sa_thr_pub)
                    else:
                        node.get_logger().warning(f"Unhandled Site: {site}")
                case {
                    "type": "sa_mode",
                    "mode": mode,
                }:
                    cur_sa_mode = mode
                case {"type": "auton_enable", "enabled": enabled, "waypoints": waypoints}:
                    self.send_auton_command(waypoints, enabled)
                case {"type": "teleop_enable", "enabled": enabled}:
                    self.enable_teleop_srv.call(SetBool.Request(data=enabled)) # SETBOOL NOT ENABLEBOOL
                case {
                    "type": "save_auton_waypoint_list",
                    "data": waypoints,
                }:
                    save_auton_waypoint_list(waypoints)
                case {
                    "type": "save_basic_waypoint_list",
                    "data": waypoints,
                }:
                    save_basic_waypoint_list(waypoints)
                case {
                    "type": "save_current_auton_course",
                    "data": waypoints
                }:
                    node.get_logger().info(f"saving waypoints in course: {waypoints}")
                    save_current_auton_course(waypoints)
                case {
                    "type": "save_current_basic_course",
                    "data": waypoints                  
                }:
                    save_current_basic_course(waypoints)
                case {
                    "type": "delete_auton_waypoint_from_course",
                    "data": waypoint
                }:
                    node.get_logger().info(f"deleting waypoint in course: {waypoint}")
                    delete_auton_waypoint_from_course(waypoint)
                case {
                    "type": "get_basic_waypoint_list",
                }:
                    self.send_message_as_json({"type": "get_basic_waypoint_list", "data": get_basic_waypoint_list()})
                case {
                    "type": "get_auton_waypoint_list",
                }:
                    self.send_message_as_json({"type": "get_auton_waypoint_list", "data": get_auton_waypoint_list()})
                case {
                    "type": "get_current_basic_course"
                }:
                    self.send_message_as_json({"type": "get_auton_waypoint_list", "data": get_current_basic_course()})
                case {
                    "type": "get_current_auton_course"
                }:
                    node.get_logger().info(f"current waypoints in course: {get_current_auton_course()}")
                    self.send_message_as_json({"type": "get_current_auton_course", "data": get_current_auton_course()})
                case {
                    "type": "heater_enable", "enable": e, "heater": heater
                }:
                    self.heater_services[heater_names.index(heater)].call(EnableBool.Request(enable=e))

                case {
                    "type": "set_gear_diff_pos",
                    "position": position,
                    "isCCW": isCCW,
                }:
                    # node.get_logger().info(f"{isCCW}")
                    self.gear_diff_set_pos_srv.call(ServoSetPos.Request(position=float(position), is_counterclockwise=isCCW))

                case {"type": "auto_shutoff", "shutoff": shutoff}:
                    self.auto_shutoff_service.call(EnableBool.Request(enable=shutoff))

                case {"type": "white_leds", "site": site, "enable": e}:
                    self.white_leds_services[site].call(EnableBool.Request(enable=e))

                case {"type": "ls_toggle", "enable": e}:
                    self.sa_enable_switch_srv.call(EnableBool.Request(enable=e))

                case _:
                    node.get_logger().warning(f"Unhandled message: {message}")
        except:
            node.get_logger().error(f"Failed to handle message: {message}")
            node.get_logger().error(traceback.format_exc())
