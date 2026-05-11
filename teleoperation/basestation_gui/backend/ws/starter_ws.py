from backend.ws.base_ws import WebSocketHandler
from backend.managers.ros import get_logger

class StarterHandler(WebSocketHandler):
    def __init__(self, websocket):
        super().__init__(websocket, 'starter')
    
    async def setup(self):
        pass

    async def handle_message(self, data):
        msg_type = data.get("type")

        if msg_type == "debug":
            get_logger().warning("debug message received by consumer at " + str(data.get("timestamp")) )
        elif msg_type == "testing":
            get_logger().warning("testing message '" + data.get("datum") + "' received at " + str(data.get("timestamp")))
        else:
            get_logger().warning(f"Unhandled Starter message: {msg_type}")