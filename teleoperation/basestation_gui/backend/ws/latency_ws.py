import asyncio
import time
from backend.ws.base_ws import WebSocketHandler

JETSON_IP = "10.1.0.10"
PING_INTERVAL_SEC = 1.0


class LatencyHandler(WebSocketHandler):
    def __init__(self, websocket):
        super().__init__(websocket, "latency")
        self._ping_task: asyncio.Task | None = None

    async def setup(self):
        self._ping_task = asyncio.create_task(self._ping_loop())

    async def _ping_loop(self):
        while True:
            latency_ms = await self._ping_jetson()
            await self.send_msgpack({"type": "jetson_ping", "latency_ms": latency_ms})
            await asyncio.sleep(PING_INTERVAL_SEC)

    async def _ping_jetson(self) -> float | None:
        try:
            proc = await asyncio.create_subprocess_exec(
                "ping", "-c", "1", "-W", "2", JETSON_IP,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.DEVNULL,
            )
            stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=3.0)
            _, sep, after = stdout.decode().partition("time=")
            return float(after.split()[0]) if sep else None
        except (asyncio.TimeoutError, OSError, ValueError):
            return None

    async def handle_message(self, data):
        if data.get("type") == "ping":
            await self.send_msgpack({
                "type": "pong",
                "timestamp": data.get("timestamp"),
                "sequence": data.get("sequence"),
                "server_time": time.time() * 1000,
                "payload": data.get("payload"),
                "processed_data": {
                    "motors": ["motor_1", "motor_2", "motor_3", "motor_4", "motor_5", "motor_6"],
                    "positions": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                    "velocities": [1.1, 1.2, 1.3, 1.4, 1.5, 1.6],
                    "efforts": [2.1, 2.2, 2.3, 2.4, 2.5, 2.6],
                    "states": ["active", "active", "active", "active", "active", "active"],
                },
            })

    async def cleanup(self):
        if self._ping_task:
            self._ping_task.cancel()
            try:
                await self._ping_task
            except asyncio.CancelledError:
                pass
        await super().cleanup()
