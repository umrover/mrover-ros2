import rclpy.time
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from lie import SE3
from backend.ws.base_ws import WebSocketHandler
from backend.managers.ros import get_logger
from backend.input import DeviceInputs
from backend.ra_controls import send_ra_controls
from mrover.msg import Throttle, IK, ControllerState
from geometry_msgs.msg import Twist
from rclpy.publisher import Publisher
class ArmHandler(WebSocketHandler):
    arm_thr_pub: Publisher
    ik_pos_pub: Publisher
    ik_vel_pub: Publisher

    def __init__(self, websocket):
        super().__init__(websocket, "arm")
        self.buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buffer, self.node, spin_thread=False)

    async def setup(self):
        self.arm_thr_pub = self.node.create_publisher(Throttle, "/arm_thr_cmd", 1)
        self.ik_pos_pub = self.node.create_publisher(IK, "/ik_pos_cmd", 1)
        self.ik_vel_pub = self.node.create_publisher(Twist, "/ik_vel_cmd", 1)
        self.publishers.extend([self.arm_thr_pub, self.ik_pos_pub, self.ik_vel_pub])


        self.forward_ros_topic("/arm_controller_state", ControllerState, "arm_state")

    async def handle_message(self, data):
        msg_type = data.get("type")

        if msg_type == "ra_controller":
            axes = data.get("axes", [])
            buttons = data.get("buttons", [])
            device_input = DeviceInputs(axes, buttons)
            send_ra_controls(
                device_input,
                self.arm_thr_pub,
                self.ik_pos_pub,
                self.ik_vel_pub,
            )
        else:
            get_logger().warning(f"Unhandled ARM message: {msg_type}")
