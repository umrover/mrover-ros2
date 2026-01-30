import asyncio
import time
import msgpack
from backend.ws.base_ws import WebSocketHandler

class LatencyHandler(WebSocketHandler):
    def __init__(self, websocket):
        super().__init__(websocket, 'latency')

    async def setup(self):
        pass

    async def handle_message(self, data):
        if data.get('type') == 'ping':
            await asyncio.sleep(0.001)

            response = {
                'type': 'pong',
                'timestamp': data.get('timestamp'),
                'sequence': data.get('sequence'),
                'server_time': time.time() * 1000,
                'payload': data.get('payload'),
                'processed_data': {
                    'motors': ['motor_1', 'motor_2', 'motor_3', 'motor_4', 'motor_5', 'motor_6'],
                    'positions': [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                    'velocities': [1.1, 1.2, 1.3, 1.4, 1.5, 1.6],
                    'efforts': [2.1, 2.2, 2.3, 2.4, 2.5, 2.6],
                    'states': ['active', 'active', 'active', 'active', 'active', 'active']
                }
            }
            await self.send_msgpack(response)
