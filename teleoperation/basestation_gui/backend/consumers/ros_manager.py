# backend/ros_manager.py
import rclpy
import threading
import atexit
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Use a lock to ensure thread-safe initialization
_lock = threading.Lock()

# Global variables for the singleton instance
_context = None
_node = None
_ros_thread = None

def get_node() -> Node:
    """Returns the singleton ROS2 node, initializing it if necessary."""
    with _lock:
        if _node is None:
            _init_ros()
        return _node

def get_context():
    """Returns the singleton ROS2 context."""
    with _lock:
        if _context is None:
            _init_ros()
        return _context

def _ros_spin():
    """Target function for the ROS thread. Spins the executor until shutdown."""
    executor = MultiThreadedExecutor(context=_context)
    executor.add_node(_node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        # The shutdown of the node and context is handled by the atexit hook

def _init_ros():
    """
    Initializes the ROS context, node, and starts the spin in a background thread.
    This function is protected by the lock in get_node/get_context.
    """
    global _context, _node, _ros_thread
    
    print("Initializing ROS Manager...")
    _context = rclpy.Context()
    rclpy.init(context=_context)
    
    _node = Node("gui_backend_node", context=_context)

    # Register the shutdown function to be called when the application exits
    atexit.register(_shutdown_ros)

    _ros_thread = threading.Thread(target=_ros_spin, daemon=True)
    _ros_thread.start()
    print("ROS Manager started in a background thread.")

def _shutdown_ros():
    """
    Called automatically on application exit to cleanly shut down ROS.
    """
    global _context, _node
    print("Shutting down ROS Manager...")
    if _context and _context.is_valid():
        if _node:
            _node.destroy_node()
        rclpy.shutdown(context=_context)