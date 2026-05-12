from backend.ws.base_ws import WebSocketHandler
from backend.managers.ros import get_logger
from rclpy.publisher import Publisher
from std_msgs.msg import String

class StarterHandler(WebSocketHandler):
    foo_pub: Publisher

    def __init__(self, websocket):
        super().__init__(websocket, 'starter')
    
    async def setup(self):
        # Create publisher
        self.foo_pub = self.node.create_publisher(String, "/foo", 1)

        # Add to list of publishers
        self.publishers.extend([self.foo_pub])

        # Forward published topic to frontend
        self.forward_ros_topic("/foo", String, "foo")

        # Do timer_callback() every 2 seconds
        timer_period = 2
        self.timer = self.node.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = "Foo has printed " + str(self.i) + " times."
        self.foo_pub.publish(msg)
        self.i += 1

    async def handle_message(self, data):
        msg_type = data.get("type")

        if msg_type == "debug":
            get_logger().warning("debug message received by consumer at " + str(data.get("timestamp")) )
        elif msg_type == "testing":
            get_logger().warning("testing message '" + str(data.get("datum")) + "' received at " + str(data.get("timestamp")))
        else:
            get_logger().warning(f"Unhandled Starter message: {msg_type}")