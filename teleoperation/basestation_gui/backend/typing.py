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
        self._action_client = ActionClient(node, KeyAction, 'KeyAction')

    def send_code(self, code):
        from backend.consumers import GUIConsumer
        if isinstance(self.websocket, GUIConsumer):
            goal_msg = KeyAction.Goal()
            goal_msg.code = code

            # self._action_client.wait_for_server()

            send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            send_goal_future.add_done_callback(self.goal_response_callback)
            
            return send_goal_future
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("goal rejected")
            return
        self.get_logger().info("goal accepted")
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_key = feedback.key
        current_state = feedback.state
        self.get_logger().info(message=f'Received feedback: {current_key}, {current_state}')
        self.websocket.send_message_as_json(
            {
                'type': 'typing_feedback',
                'current_key': current_key, 
                'current_state': current_state,
            }
        )

    def shutdown(self):
        # Destroy the node properly
        self.destroy_node()
