import rclpy
from rclpy.node import Node
from threading import Lock

_node = None
_context = None
_lock = Lock()

def get_node():
    global _node, _context
    with _lock:
        if _node is None:
            _context = rclpy.Context()
            _context.init()
            _node = rclpy.create_node("teleoperation", context=_context)
        return _node

def get_context():
    return _context
