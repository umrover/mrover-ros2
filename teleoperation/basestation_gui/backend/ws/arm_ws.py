import rclpy.time
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from lie import SE3
from backend.ws.base_ws import WebSocketHandler
from backend.managers.ros import get_logger
from backend.input import DeviceInputs
from backend.ra_controls import send_ra_controls, register_ik_pos_pub
from mrover.msg import Throttle, IK, ControllerState
from geometry_msgs.msg import Twist
from rclpy.publisher import Publisher


class ArmHandler(WebSocketHandler):
    arm_thr_pub: Publisher
    ik_pos_pub: Publisher
    ik_vel_pub: Publisher

    def __init__(self, websocket):
        super().__init__(websocket, 'arm')
        self.buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buffer, self.node, spin_thread=False)

    async def setup(self):
        self.arm_thr_pub = self.node.create_publisher(Throttle, "/arm_thr_cmd", 1)
        self.ik_pos_pub = self.node.create_publisher(IK, "/ik_pos_cmd", 1)
        self.ik_vel_pub = self.node.create_publisher(Twist, "/ik_vel_cmd", 1)
        self.publishers.extend([self.arm_thr_pub, self.ik_pos_pub, self.ik_vel_pub])
        register_ik_pos_pub(self.ik_pos_pub)

        self.forward_ros_topic("/arm_controller_state", ControllerState, "arm_state")
        self.forward_ros_topic("/arm_ik", IK, "ik_target")
        self.forward_ros_topic("/arm_thr_cmd", Throttle, "arm_throttle_command")

        self.timers.append(self.node.create_timer(0.1, self.send_arm_feedback_callback))

    def send_arm_feedback_callback(self):
        try:
            if not self.buffer.can_transform("arm_base_link", "arm_fk", rclpy.time.Time()):
                return
            arm_in_base = SE3.from_tf_tree(self.buffer, "arm_base_link", "arm_fk")
            pos = arm_in_base.translation()
            self.schedule_send({
                "type": "ik_feedback",
                "pos": {
                    "x": float(pos[0]),
                    "y": float(pos[1]),
                    "z": float(pos[2]),
                },
            })
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
        except Exception as e:
            get_logger().error(f"ArmHandler feedback error: {e}")

    async def handle_message(self, data):
        msg_type = data.get('type')

        if msg_type == 'ra_controller':
            axes = data.get('axes', [])
            buttons = data.get('buttons', [])
            device_input = DeviceInputs(axes, buttons)
            send_ra_controls(
                device_input,
                self.arm_thr_pub,
                self.ik_pos_pub,
                self.ik_vel_pub,
            )
        else:
            get_logger().warning(f"Unhandled ARM message: {msg_type}")
