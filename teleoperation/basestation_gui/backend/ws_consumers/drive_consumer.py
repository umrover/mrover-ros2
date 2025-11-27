from backend.ws_consumers.base_consumer import WebSocketHandler
from backend.input import DeviceInputs
from backend.drive_controls import send_joystick_twist, send_controller_twist
from mrover.msg import ControllerState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class DriveConsumer(WebSocketHandler):
    def __init__(self, websocket):
        super().__init__(websocket, 'drive')

    async def setup(self):
        """Setup DRIVE endpoint subscriptions and publishers"""
        self.joystick_twist_pub = self.node.create_publisher(Twist, "/joystick_cmd_vel", 1)
        self.controller_twist_pub = self.node.create_publisher(Twist, "/controller_cmd_vel", 1)

        self.forward_ros_topic("/drive_left_controller_data", ControllerState, "drive_left_state")
        self.forward_ros_topic("/drive_right_controller_data", ControllerState, "drive_right_state")
        self.forward_ros_topic("/drive_left_joint_data", JointState, "drive_left_joint_state")
        self.forward_ros_topic("/drive_right_joint_data", JointState, "drive_right_joint_state")

    async def handle_message(self, data):
        """Handle incoming DRIVE control messages"""
        msg_type = data.get('type')

        if msg_type == 'joystick':
            axes = data.get('axes', [])
            buttons = data.get('buttons', [])
            device_input = DeviceInputs(axes, buttons)
            send_joystick_twist(device_input, self.joystick_twist_pub)
        elif msg_type == 'controller':
            axes = data.get('axes', [])
            buttons = data.get('buttons', [])
            device_input = DeviceInputs(axes, buttons)
            send_controller_twist(device_input, self.controller_twist_pub)
        else:
            print(f"Unhandled DRIVE message: {msg_type}")
