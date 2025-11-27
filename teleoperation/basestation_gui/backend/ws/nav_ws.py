import asyncio
import tf2_ros
from lie import SE3
from backend.ws.base_ws import WebSocketHandler
from mrover.msg import StateMachineStateUpdate
from sensor_msgs.msg import NavSatFix

class NavHandler(WebSocketHandler):
    def __init__(self, websocket):
        super().__init__(websocket, 'nav')
        self.buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buffer, self.node, spin_thread=False)

    async def setup(self):
        """Setup NAV endpoint subscriptions and timers"""
        self.forward_ros_topic("/nav_state", StateMachineStateUpdate, "nav_state")
        self.forward_ros_topic("/gps/fix", NavSatFix, "gps_fix")
        self.forward_ros_topic("basestation/position", NavSatFix, "basestation_position")
        self.forward_ros_topic("/drone_odometry", NavSatFix, "drone_waypoint")

        # Create timer for localization updates
        timer = self.node.create_timer(0.1, self.send_localization_callback)  # 10 Hz
        self.timers.append(timer)

    def send_localization_callback(self):
        """Send orientation from TF tree"""
        try:
            base_link_in_map = SE3.from_tf_tree(self.buffer, "map", "base_link")
            quat = base_link_in_map.quat().tolist()
            data_to_send = {
                "type": "orientation",
                "orientation": {
                    "x": quat[0],
                    "y": quat[1],
                    "z": quat[2],
                    "w": quat[3],
                },
            }
            asyncio.run_coroutine_threadsafe(self.send_msgpack(data_to_send), self.loop)
        except Exception:
            # Errors are expected if localization isn't running
            pass

    async def handle_message(self, data):
        """Handle incoming NAV messages"""
        print(f"Nav handler received message: {data}")
