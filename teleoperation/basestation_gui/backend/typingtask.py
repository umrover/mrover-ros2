"""ROS2 Action Client for the Typing Task"""

import rclpy
from mrover.action import KeyAction
from rclpy.action import ActionClient
from rclpy.node import Node

class TypingTaskActionClient(Node):
    # may need a better way to define the type of the websocket
    def __init__(self, node: Node, websocket: any):
        super().__init__('typing_task_action_client')
        self.node = node
        self.websocket = websocket  # Instance of GUIConsumer
        self._action_client = ActionClient(node, KeyAction, 'key_action')

    def send_code(self, code):
        # Delay importing GUIConsumer to avoid circular dependency
        from backend.consumers import GUIConsumer
        # self.get_logger().info(f"code in typingtask client: {code}")
        if isinstance(self.websocket, GUIConsumer):
            goal_msg = KeyAction.Goal()
            goal_msg.code = code
            self.node.get_logger().info(f"websocket found! Sending code")

        # self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
    
    def feedback_callback(self, feedback_msg):
        current_key = feedback_msg.key
        current_state = feedback_msg.state
        # self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))
        self.websocket.send_message_as_json(
            {'currentKey': current_key, 
                 'currentState': current_state}
        )

    def shutdown(self):
        # Destroy the node properly
        self.destroy_node()
