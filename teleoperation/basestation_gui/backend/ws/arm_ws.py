from typing import Optional
from backend.ws.base_ws import WebSocketHandler
from backend.input import DeviceInputs
from backend.ra_controls import send_ra_controls
from mrover.msg import Throttle, IK, ControllerState, Position, Velocity
from geometry_msgs.msg import Twist
from rclpy.publisher import Publisher

class ArmHandler(WebSocketHandler):
    arm_thr_pub: Publisher
    ik_pos_pub: Publisher
    ik_vel_pub: Publisher

    def __init__(self, websocket):
        super().__init__(websocket, 'arm')
        self.buffer = {}

    async def setup(self):
        """Setup ARM endpoint subscriptions and publishers"""
        self.arm_thr_pub = self.node.create_publisher(Throttle, "/arm_thr_cmd", 1)
        self.ik_pos_pub = self.node.create_publisher(IK, "/ik_pos_cmd", 1)
        self.ik_vel_pub = self.node.create_publisher(Twist, "/ik_vel_cmd", 1)

        self.forward_ros_topic("/arm_controller_state", ControllerState, "arm_state")
        self.forward_ros_topic("/arm_ik", IK, "ik_target")

    async def handle_message(self, data):
        """Handle incoming ARM control messages"""
        msg_type = data.get('type')

        if msg_type == 'ra_controller':
            axes = data.get('axes', [])
            buttons = data.get('buttons', [])
            device_input = DeviceInputs(axes, buttons)
            send_ra_controls(
                device_input,
                self.node,
                self.arm_thr_pub,
                self.ik_pos_pub,
                self.ik_vel_pub,
                self.buffer,
            )
        else:
            print(f"Unhandled ARM message: {msg_type}")
