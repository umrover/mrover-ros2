import asyncio
from backend.ws.base_ws import WebSocketHandler
from backend.managers.ros import get_logger
from rclpy.action import ActionClient
from mrover.action import TypingCode


class AutonHandler(WebSocketHandler):
    def __init__(self, websocket):
        super().__init__(websocket, 'auton')
        self.current_goal_handle = None

    async def setup(self):
        self.action_client = ActionClient(self.node, TypingCode, '/es_typing_code')

    async def handle_message(self, data):
        msg_type = data.get('type')

        if msg_type == 'code':
            code = data.get('code', '')
            if code == 'cancel':
                await self.cancel_goal()
            else:
                await self.send_typing_code(code)
        else:
            get_logger().warning(f"Unhandled AUTON message: {msg_type}")

    async def send_typing_code(self, code: str):
        server_ready = await asyncio.to_thread(
            self.action_client.wait_for_server, timeout_sec=1.0
        )
        if not server_ready:
            self.schedule_send({'type': 'typing_error', 'error': 'Action server not available'})
            return

        goal_msg = TypingCode.Goal()
        goal_msg.launch_code = code

        future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.schedule_send({'type': 'typing_error', 'error': 'Goal rejected'})
            return
        self.current_goal_handle = goal_handle
        self.schedule_send({'type': 'typing_accepted'})

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.schedule_send({
            'type': 'typing_feedback',
            'current_state': feedback.current_state,
            'current_index': feedback.current_index,
        })

    async def cancel_goal(self):
        if self.current_goal_handle:
            await asyncio.to_thread(self.current_goal_handle.cancel_goal_async)
            self.current_goal_handle = None
            self.schedule_send({'type': 'typing_cancelled'})

    async def cleanup(self):
        if self.current_goal_handle:
            try:
                await asyncio.to_thread(self.current_goal_handle.cancel_goal_async)
            except Exception:
                pass
        if hasattr(self, 'action_client'):
            self.action_client.destroy()
        await super().cleanup()
