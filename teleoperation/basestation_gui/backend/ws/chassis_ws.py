from backend.ws.base_ws import WebSocketHandler
from mrover.msg import ControllerState

class ChassisHandler(WebSocketHandler):
    def __init__(self, websocket):
        super().__init__(websocket, 'chassis')

    async def setup(self):
        self.forward_ros_topic("/gimbal_controller_state", ControllerState, "gimbal_controller_state")

    async def handle_message(self, data):
        print(f"Chassis handler received unexpected message: {data}")
