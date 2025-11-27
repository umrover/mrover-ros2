import asyncio
import time
import threading
import os
import sys
from flask import Flask, jsonify
from flask_cors import CORS
import websockets
import msgpack

# Import ROS manager - add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from backend.consumers.ros_manager import get_node

# Import Blueprints
from backend.routes.waypoints import waypoints_bp
from backend.routes.auton import auton_bp
from backend.routes.mast import mast_bp
from backend.routes.science import science_bp

# Import Consumers
from backend.ws_consumers.arm_consumer import ArmConsumer
from backend.ws_consumers.drive_consumer import DriveConsumer
from backend.ws_consumers.mast_consumer import MastConsumer
from backend.ws_consumers.nav_consumer import NavConsumer
from backend.ws_consumers.science_consumer import ScienceConsumer
from backend.ws_consumers.latency_consumer import LatencyConsumer

app = Flask(__name__)
CORS(app)

# Register Blueprints
app.register_blueprint(waypoints_bp, url_prefix='/api/waypoints')
app.register_blueprint(auton_bp, url_prefix='/api')
app.register_blueprint(mast_bp, url_prefix='/api')
app.register_blueprint(science_bp, url_prefix='/api')

@app.route('/api/health', methods=['GET'])
def health():
    return jsonify({'status': 'ok'})

async def websocket_handler(websocket):
    """Route WebSocket connections based on path"""
    path = websocket.request.path
    print(f"Client connected: {path}")

    handler = None
    if path == '/latency':
        handler = LatencyConsumer(websocket)
    elif path == '/arm':
        handler = ArmConsumer(websocket)
    elif path == '/drive':
        handler = DriveConsumer(websocket)
    elif path == '/mast':
        handler = MastConsumer(websocket)
    elif path == '/nav':
        handler = NavConsumer(websocket)
    elif path == '/science':
        handler = ScienceConsumer(websocket)
    else:
        print(f"Unknown WebSocket endpoint: {path}")
        await websocket.close()
        return

    try:
        await handler.setup()
        async for message in websocket:
            data = msgpack.unpackb(message, raw=False)
            await handler.handle_message(data)
    except websockets.exceptions.ConnectionClosed:
        print(f"Client disconnected: {path}")
    except Exception as e:
        print(f"Error in {path} handler: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if handler:
            await handler.cleanup()

async def websocket_server():
    """Run WebSocket server on port 8001"""
    async with websockets.serve(websocket_handler, "0.0.0.0", 8001):
        print("WebSocket server running on ws://0.0.0.0:8001")
        print("Available endpoints: /arm, /drive, /mast, /nav, /science, /latency")
        await asyncio.Future()

def run_flask():
    """Run Flask REST API on port 8000"""
    app.run(host='0.0.0.0', port=8000, debug=False, use_reloader=False)

def run_websocket():
    """Run WebSocket server in asyncio event loop"""
    asyncio.run(websocket_server())

if __name__ == '__main__':
    print("Starting MRover Basestation Backend...")

    # Initialize ROS through ros_manager
    node = get_node()
    print("ROS2 node initialized")

    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    print("Flask REST API starting on http://0.0.0.0:8000")

    try:
        run_websocket()
    except KeyboardInterrupt:
        print("\nShutting down...")