import rclpy
import threading
import atexit
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

_lock = threading.Lock()
_initialized = threading.Event()

_context = None
_node = None
_ros_thread = None
  
def get_node() -> Node:
    """Returns the singleton ROS2 node, initializing it if necessary."""
    if not _initialized.is_set():
        with _lock:
            if not _initialized.is_set():  # Double-checked locking
                _init_ros()
    return _node

def get_logger():
    """Returns the ROS2 logger from the singleton node."""
    return get_node().get_logger()

def get_context():
    """Returns the singleton ROS2 context."""
    if not _initialized.is_set():
        with _lock:
            if not _initialized.is_set():
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

def _init_ros():
    """
    Initializes the ROS context, node, and starts the spin in a background thread.
    This function is protected by the lock and guarded by an event.
    """
    global _context, _node, _ros_thread

    print("Initializing ROS Manager...")
    _context = rclpy.Context()
    rclpy.init(context=_context)
    
    _node = Node("gui_backend_node", context=_context)
    atexit.register(_shutdown_ros)

    _ros_thread = threading.Thread(target=_ros_spin, daemon=True)
    _ros_thread.start()

    print("ROS Manager started in a background thread.")
    _initialized.set()  # Mark as initialized

def _shutdown_ros():
    """
    Called automatically on application exit to cleanly shut down ROS.
    """
    global _context, _node
    print("Shutting down ROS Manager...")
    if _context and rclpy.ok(context=_context):
        if _node:
            _node.destroy_node()
        rclpy.shutdown(context=_context)
