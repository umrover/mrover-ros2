from backend.ws_consumers.base_consumer import WebSocketHandler
from backend.input import DeviceInputs
from backend.sp_controls import send_sp_controls
from mrover.msg import Throttle, ControllerState, LED, Oxygen, UV
from sensor_msgs.msg import Temperature, RelativeHumidity, JointState

class ScienceConsumer(WebSocketHandler):
    def __init__(self, websocket):
        super().__init__(websocket, 'science')

    async def setup(self):
        """Setup SCIENCE endpoint subscriptions and publishers"""
        self.sp_thr_pub = self.node.create_publisher(Throttle, "/sp_throttle_cmd", 1)

        self.forward_ros_topic("/led", LED, "led")
        self.forward_ros_topic("/science_oxygen_data", Oxygen, "oxygen")
        self.forward_ros_topic("/science_uv_data", UV, "uv")
        self.forward_ros_topic("/science_temperature_data", Temperature, "temperature")
        self.forward_ros_topic("/science_humidity_data", RelativeHumidity, "humidity")
        self.forward_ros_topic("/sp_joint_state", JointState, "sp_joint_state")
        self.forward_ros_topic("/sp_controller_state", ControllerState, "sp_state")

    async def handle_message(self, data):
        """Handle incoming SCIENCE control messages"""
        msg_type = data.get('type')

        if msg_type == 'sp_controller':
            axes = data.get('axes', [])
            buttons = data.get('buttons', [])
            device_input = DeviceInputs(axes, buttons)
            send_sp_controls(device_input, self.sp_thr_pub)
        else:
            print(f"Unhandled SCIENCE message: {msg_type}")
