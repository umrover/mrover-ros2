import rclpy
import threading
import atexit
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

lock = threading.Lock()
initialized = threading.Event()

context = None
node = None
ros_thread = None
service_clients = {}
service_clients_lock = threading.Lock()


def get_node() -> Node:
    if not initialized.is_set():
        with lock:
            if not initialized.is_set():
                init_ros()
    return node


def get_logger():
    return get_node().get_logger()


def get_service_client(srv_type, srv_name):
    with service_clients_lock:
        if srv_name not in service_clients:
            n = get_node()
            service_clients[srv_name] = n.create_client(srv_type, srv_name)
        return service_clients[srv_name]


def get_context():
    if not initialized.is_set():
        with lock:
            if not initialized.is_set():
                init_ros()
    return context


def ros_spin():
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()


def init_ros():
    global context, node, ros_thread

    print("Initializing ROS Manager...")
    context = rclpy.Context()
    rclpy.init(context=context)

    node = Node("gui_backend_node", context=context)
    atexit.register(shutdown_ros)

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    print("ROS Manager started in a background thread.")
    initialized.set()


def shutdown_ros():
    global context, node
    print("Shutting down ROS Manager...")
    if context and rclpy.ok(context=context):
        if node:
            node.destroy_node()
        rclpy.shutdown(context=context)
