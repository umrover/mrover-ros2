from backend.ws.base_ws import WebSocketHandler
from backend.managers.ros import get_logger
from backend.input import DeviceInputs
from backend.drive_controls import send_joystick_twist, send_controller_twist
from mrover.msg import ControllerState
from geometry_msgs.msg import Twist


class DriveHandler(WebSocketHandler):
    def __init__(self, websocket):
        super().__init__(websocket, 'drive')

    async def setup(self):
        self.joystick_twist_pub = self.node.create_publisher(Twist, "/joystick_vel_cmd", 1)
        self.controller_twist_pub = self.node.create_publisher(Twist, "/controller_vel_cmd", 1)
        self.publishers.extend([self.joystick_twist_pub, self.controller_twist_pub])

        self.forward_ros_topic("/left_controller_state", ControllerState, "drive_left_state")
        self.forward_ros_topic("/right_controller_state", ControllerState, "drive_right_state")

    async def handle_message(self, data):
        msg_type = data.get('type')

        if msg_type == 'joystick':
            axes = data.get('axes', [])
            buttons = data.get('buttons', [])
            send_joystick_twist(DeviceInputs(axes, buttons), self.joystick_twist_pub)
        elif msg_type == 'controller':
            axes = data.get('axes', [])
            buttons = data.get('buttons', [])
            send_controller_twist(DeviceInputs(axes, buttons), self.controller_twist_pub)
        else:
            get_logger().warning(f"Unhandled DRIVE message: {msg_type}")
