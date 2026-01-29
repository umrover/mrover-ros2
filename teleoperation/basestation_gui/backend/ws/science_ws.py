import asyncio
from backend.ws.base_ws import WebSocketHandler
from backend.input import DeviceInputs
from backend.sp_controls import send_sp_controls
from mrover.msg import Throttle, ControllerState, Humidity, Temperature, Oxygen, UV, Ozone, CO2, Pressure

class ScienceHandler(WebSocketHandler):
    def __init__(self, websocket):
        super().__init__(websocket, 'science')

    async def setup(self):
        self.sp_thr_pub = self.node.create_publisher(Throttle, "/sp_thr_cmd", 1)
        self.publishers.append(self.sp_thr_pub)

        self.forward_ros_topic("/sp_humidity_data", Humidity, "sp_humidity")
        self.forward_ros_topic("/sp_temp_data", Temperature, "sp_temp")
        self.forward_ros_topic("/sp_oxygen_data", Oxygen, "sp_oxygen")
        self.forward_ros_topic("/sp_uv_data", UV, "sp_uv")
        self.forward_ros_topic("/sp_ozone_data", Ozone, "sp_ozone")
        self.forward_ros_topic("/sp_co2_data", CO2, "sp_co2")
        self.forward_ros_topic("/sp_pressure_data", Pressure, "sp_pressure")
        self.forward_ros_topic("/sp_controller_state", ControllerState, "sp_controller_state")

    async def handle_message(self, data):
        msg_type = data.get('type')

        if msg_type == 'sp_controller':
            axes = data.get('axes', [])
            buttons = data.get('buttons', [])
            device_input = DeviceInputs(axes, buttons)
            await asyncio.to_thread(send_sp_controls, device_input, self.sp_thr_pub)
        else:
            print(f"Unhandled SCIENCE message: {msg_type}")
