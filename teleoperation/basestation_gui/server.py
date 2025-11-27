import asyncio
import sys
import os
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
import msgpack

# Import ROS manager - add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from backend.ros_manager import get_node

# Import Routers
from backend.routes.waypoints import router as waypoints_router
from backend.routes.recordings import router as recordings_router
from backend.routes.auton import router as auton_router
from backend.routes.chassis import router as chassis_router
from backend.routes.science import router as science_router
from backend.routes.arm import router as arm_router

# Import Handlers
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
    allow_credentials=True,
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

@app.get("/api/health")
def health():
    return {"status": "ok"}

# WebSocket Handlers ... (Keep existing logic)
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
    # Initialize ROS
    node = get_node()
    print("ROS2 node initialized")

    # Serve Frontend Static Files (Production Mode)
    if os.getenv("SERVE_STATIC_FRONTEND") == "true":
        frontend_dist = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frontend/dist")
        if os.path.exists(frontend_dist):
            print(f"Serving static files from {frontend_dist}")
            # Serve files from assets directory first (css, js, images)
            app.mount("/assets", StaticFiles(directory=os.path.join(frontend_dist, "assets")), name="assets")
            
            # Catch-all route for SPA (Single Page Application) support
            # This must be the last route registered.
            @app.get("/{full_path:path}")
            async def serve_spa(full_path: str):
                # Check if the requested path is for a file that exists in the root of dist
                file_path = os.path.join(frontend_dist, full_path)
                if os.path.exists(file_path) and os.path.isfile(file_path):
                    return FileResponse(file_path)
                    
                # Otherwise, serve index.html for SPA routing (e.g., /about -> index.html)
                return FileResponse(os.path.join(frontend_dist, "index.html"))
        else:
            print("Frontend 'dist' directory not found. Not serving static files.")
    else:
        print("SERVE_STATIC_FRONTEND is not 'true'. Running in API-only mode (no static file serving).")

    # Run Uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
