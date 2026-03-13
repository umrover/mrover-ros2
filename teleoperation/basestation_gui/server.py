import argparse
import sys
import os
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
import msgpack
import traceback

# Import Singleton ROS Manager
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from backend.managers.ros import get_node, get_logger
from backend.logging_config import LOGGING_CONFIG

# API Routers
from backend.routes.waypoints import router as waypoints_router
from backend.routes.recordings import router as recordings_router
from backend.routes.auton import router as auton_router
from backend.routes.chassis import router as chassis_router
from backend.routes.science import router as science_router
from backend.routes.arm import router as arm_router

# Websocket Handlers
from backend.ws.arm_ws import ArmHandler
from backend.ws.drive_ws import DriveHandler
from backend.ws.chassis_ws import ChassisHandler
from backend.ws.nav_ws import NavHandler
from backend.ws.science_ws import ScienceHandler
from backend.ws.latency_ws import LatencyHandler

app = FastAPI()

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include Routers
app.include_router(waypoints_router)
app.include_router(recordings_router)
app.include_router(auton_router)
app.include_router(chassis_router)
app.include_router(science_router)
app.include_router(arm_router)

MAX_WS_PAYLOAD_BYTES = 1024 * 1024  # 1 MB

# WebSocket Handlers
async def handle_websocket(websocket: WebSocket, ConsumerClass):
    await websocket.accept()
    handler = ConsumerClass(websocket)
    try:
        await handler.setup()
        while True:
            data = await websocket.receive_bytes()
            if len(data) > MAX_WS_PAYLOAD_BYTES:
                get_logger().warning(f"Oversized payload ({len(data)} bytes) on {handler.endpoint}, dropping")
                continue
            unpacked = msgpack.unpackb(data, raw=False)
            await handler.handle_message(unpacked)
    except WebSocketDisconnect:
        pass
    except Exception as e:
        get_logger().error(f"Error in {handler.endpoint} handler: {e}\n{traceback.format_exc()}")
    finally:
        await handler.cleanup()

@app.websocket("/arm")
async def ws_arm(websocket: WebSocket):
    await handle_websocket(websocket, ArmHandler)

@app.websocket("/drive")
async def ws_drive(websocket: WebSocket):
    await handle_websocket(websocket, DriveHandler)

@app.websocket("/chassis")
async def ws_chassis(websocket: WebSocket):
    await handle_websocket(websocket, ChassisHandler)

@app.websocket("/nav")
async def ws_nav(websocket: WebSocket):
    await handle_websocket(websocket, NavHandler)

@app.websocket("/science")
async def ws_science(websocket: WebSocket):
    await handle_websocket(websocket, ScienceHandler)

@app.websocket("/latency")
async def ws_latency(websocket: WebSocket):
    await handle_websocket(websocket, LatencyHandler)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--serve-static", action="store_true")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    get_node()

    if args.serve_static:
        frontend_dist = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frontend/dist")
        real_frontend_dist = os.path.realpath(frontend_dist)
        app.mount("/assets", StaticFiles(directory=os.path.join(frontend_dist, "assets")), name="assets")

        @app.get("/{full_path:path}")
        async def serve_spa(full_path: str):
            file_path = os.path.realpath(os.path.join(frontend_dist, full_path))
            if not file_path.startswith(real_frontend_dist):
                return FileResponse(os.path.join(frontend_dist, "index.html"))
            if os.path.exists(file_path) and os.path.isfile(file_path):
                return FileResponse(file_path)
            return FileResponse(os.path.join(frontend_dist, "index.html"))

    uvicorn.run(app, host="0.0.0.0", port=args.port, log_config=LOGGING_CONFIG)
