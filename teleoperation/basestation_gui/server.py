import asyncio
import sys
import os
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import msgpack

# Import ROS manager - add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from backend.ros_manager import get_node

# Import Routers
from backend.routes.waypoints import router as waypoints_router
from backend.routes.auton import router as auton_router
from backend.routes.mast import router as mast_router
from backend.routes.science import router as science_router

# Import Consumers
from backend.ws_consumers.arm_consumer import ArmConsumer
from backend.ws_consumers.drive_consumer import DriveConsumer
from backend.ws_consumers.mast_consumer import MastConsumer
from backend.ws_consumers.nav_consumer import NavConsumer
from backend.ws_consumers.science_consumer import ScienceConsumer
from backend.ws_consumers.latency_consumer import LatencyConsumer

app = FastAPI()

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include Routers
app.include_router(waypoints_router)
app.include_router(auton_router)
app.include_router(mast_router)
app.include_router(science_router)

@app.get("/api/health")
def health():
    return {"status": "ok"}

async def handle_websocket(websocket: WebSocket, ConsumerClass):
    await websocket.accept()
    handler = ConsumerClass(websocket)
    try:
        await handler.setup()
        while True:
            data = await websocket.receive_bytes()
            unpacked = msgpack.unpackb(data, raw=False)
            await handler.handle_message(unpacked)
    except WebSocketDisconnect:
        print(f"Client disconnected from {handler.endpoint}")
    except Exception as e:
        print(f"Error in {handler.endpoint} handler: {e}")
        import traceback
        traceback.print_exc()
    finally:
        await handler.cleanup()

@app.websocket("/arm")
async def ws_arm(websocket: WebSocket):
    await handle_websocket(websocket, ArmConsumer)

@app.websocket("/drive")
async def ws_drive(websocket: WebSocket):
    await handle_websocket(websocket, DriveConsumer)

@app.websocket("/mast")
async def ws_mast(websocket: WebSocket):
    await handle_websocket(websocket, MastConsumer)

@app.websocket("/nav")
async def ws_nav(websocket: WebSocket):
    await handle_websocket(websocket, NavConsumer)

@app.websocket("/science")
async def ws_science(websocket: WebSocket):
    await handle_websocket(websocket, ScienceConsumer)

@app.websocket("/latency")
async def ws_latency(websocket: WebSocket):
    await handle_websocket(websocket, LatencyConsumer)

if __name__ == "__main__":
    # Initialize ROS
    node = get_node()
    print("ROS2 node initialized")
    
    # Run Uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
