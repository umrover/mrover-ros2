from backend.ws_consumers.base_consumer import WebSocketHandler
from sensor_msgs.msg import JointState

class MastConsumer(WebSocketHandler):
    def __init__(self, websocket):
        super().__init__(websocket, 'mast')

    async def setup(self):
        """Setup MAST endpoint subscriptions"""
        self.forward_ros_topic("/gimbal_joint_state", JointState, "gimbal_joint_state")

    async def handle_message(self, data):
        """Handle incoming MAST messages"""
        print(f"Mast consumer received unexpected message: {data}")
