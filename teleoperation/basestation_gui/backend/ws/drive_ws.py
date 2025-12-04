from backend.ws.base_ws import WebSocketHandler
from backend.input import DeviceInputs
from backend.drive_controls import send_joystick_twist, send_controller_twist
from mrover.msg import ControllerState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

DRIVE_PUBLISH_RATE_HZ = 60

class DriveHandler(WebSocketHandler):
    def __init__(self, websocket):
        super().__init__(websocket, 'drive')
        self.latest_joystick_input = None
        self.latest_controller_input = None

    async def setup(self):
        """Setup DRIVE endpoint subscriptions and publishers"""
        self.joystick_twist_pub = self.node.create_publisher(Twist, "/joystick_vel_cmd", 1)
        self.controller_twist_pub = self.node.create_publisher(Twist, "/controller_vel_cmd", 1)

        self.forward_ros_topic("/left_controller_state", ControllerState, "drive_left_state")
        self.forward_ros_topic("/right_controller_state", ControllerState, "drive_right_state")

        drive_timer = self.node.create_timer(1.0 / DRIVE_PUBLISH_RATE_HZ, self.publish_drive_commands)
        self.timers.append(drive_timer)

    def publish_drive_commands(self):
        """Publish latest drive commands at fixed rate"""
        if self.latest_joystick_input:
            send_joystick_twist(self.latest_joystick_input, self.joystick_twist_pub)
        if self.latest_controller_input:
            send_controller_twist(self.latest_controller_input, self.controller_twist_pub)

    async def handle_message(self, data):
        """Handle incoming DRIVE control messages"""
        msg_type = data.get('type')

        if msg_type == 'joystick':
            axes = data.get('axes', [])
            buttons = data.get('buttons', [])
            self.latest_joystick_input = DeviceInputs(axes, buttons)
        elif msg_type == 'controller':
            axes = data.get('axes', [])
            buttons = data.get('buttons', [])
            self.latest_controller_input = DeviceInputs(axes, buttons)
        else:
            print(f"Unhandled DRIVE message: {msg_type}")
