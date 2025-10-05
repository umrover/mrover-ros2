import rclpy
import asyncio
import atexit
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

_context = None
_node = None
_executor_task = None

def get_node() -> Node:
    """Returns the singleton ROS2 node."""
    if _node is None:
        raise RuntimeError("ROS not initialized. Call start_ros_executor() first.")
    return _node

def get_context():
    """Returns the singleton ROS2 context."""
    if _context is None:
        raise RuntimeError("ROS not initialized. Call start_ros_executor() first.")
    return _context

async def start_ros_executor():
    """Initialize ROS and start executor as an async background task."""
    global _context, _node, _executor_task

    print("Initializing ROS Manager...")
    _context = rclpy.Context()
    rclpy.init(context=_context)
    _node = Node("gui_backend_node", context=_context)
    atexit.register(_shutdown_ros)

    # Start executor as async task in the event loop
    _executor_task = asyncio.create_task(_ros_spin_async())
    print("ROS Manager started in asyncio event loop.")

async def _ros_spin_async():
    """Spin ROS executor in the asyncio event loop."""
    executor = SingleThreadedExecutor(context=_context)
    executor.add_node(_node)
    try:
        while rclpy.ok(context=_context):
            executor.spin_once(timeout_sec=0)
            await asyncio.sleep(0)  # Yield to event loop
    finally:
        executor.shutdown()

def _shutdown_ros():
    """Called automatically on application exit to cleanly shut down ROS."""
    global _context, _node
    print("Shutting down ROS Manager...")
    if _context and _context.is_valid():
        if _node:
            _node.destroy_node()
        rclpy.shutdown(context=_context)
