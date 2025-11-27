from backend.ws_consumers.base_consumer import WebSocketHandler
from backend.input import DeviceInputs
from backend.ra_controls import send_ra_controls
from mrover.msg import Throttle, IK, ControllerState, Position, Velocity
from sensor_msgs.msg import JointState

class ArmConsumer(WebSocketHandler):
    def __init__(self, websocket):
        super().__init__(websocket, 'arm')
        self.cur_ra_mode = "disabled"
        self.buffer = {}

    async def setup(self):
        """Setup ARM endpoint subscriptions and publishers"""
        self.arm_thr_pub = self.node.create_publisher(Throttle, "arm_throttle_cmd", 1)
        self.arm_pos_pub = self.node.create_publisher(Position, "/arm_position_cmd", 1)
        self.arm_vel_pub = self.node.create_publisher(Velocity, "/arm_velocity_cmd", 1)

        self.forward_ros_topic("/arm_controller_state", ControllerState, "arm_state")
        self.forward_ros_topic("/arm_joint_data", JointState, "fk")
        self.forward_ros_topic("/arm_ik", IK, "ik_target")

    async def handle_message(self, data):
        """Handle incoming ARM control messages"""
        msg_type = data.get('type')

        if msg_type == 'ra_controller':
            axes = data.get('axes', [])
            buttons = data.get('buttons', [])
            device_input = DeviceInputs(axes, buttons)
            send_ra_controls(
                self.cur_ra_mode,
                device_input,
                self.node,
                self.arm_thr_pub,
                self.arm_pos_pub,
                self.arm_vel_pub,
                self.buffer,
            )
        elif msg_type == 'ra_mode':
            self.cur_ra_mode = data.get('mode', 'disabled')
        else:
            print(f"Unhandled ARM message: {msg_type}")
