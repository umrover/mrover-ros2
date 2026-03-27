import rclpy.time
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from lie import SE3
from backend.ws.base_ws import WebSocketHandler
from backend.managers.ros import get_logger
from backend.managers.led import set_nav_state
from mrover.msg import StateMachineStateUpdate
from sensor_msgs.msg import NavSatFix


class NavHandler(WebSocketHandler):
    def __init__(self, websocket):
        super().__init__(websocket, 'nav')
        self.buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buffer, self.node, spin_thread=False)

    async def setup(self):
        self.nav_state_sub = self.node.create_subscription(
            StateMachineStateUpdate, "/nav_state", self.nav_state_callback, 10
        )
        self.subscriptions.append(self.nav_state_sub)

        self.forward_ros_topic("/gps/fix", NavSatFix, "gps_fix")
        self.forward_ros_topic("basestation/position", NavSatFix, "basestation_position")
        self.forward_ros_topic("/drone_odometry", NavSatFix, "drone_waypoint")

        timer = self.node.create_timer(0.1, self.send_localization_callback)
        self.timers.append(timer)

    def send_localization_callback(self):
        try:
            if not self.buffer.can_transform("map", "base_link", rclpy.time.Time()):
                return
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
            self.schedule_send(data_to_send)
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
        except Exception as e:
            get_logger().error(f"NavHandler localization error: {e}")

    def nav_state_callback(self, msg):
        set_nav_state(msg.state)
        data_to_send = {"type": "nav_state", "state": msg.state}
        self.schedule_send(data_to_send)

    async def handle_message(self, data):
        get_logger().info(f"Nav handler received message: {data}")
