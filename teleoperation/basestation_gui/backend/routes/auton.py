from flask import Blueprint, jsonify, request
from backend.consumers.ros_manager import get_node
from mrover.srv import EnableAuton
from mrover.msg import GPSWaypoint, WaypointType
from std_srvs.srv import SetBool
import time
import traceback

auton_bp = Blueprint('auton', __name__)

def _call_service_sync(client, request, timeout=5.0, logger=None):
    if not client.wait_for_service(timeout_sec=1.0):
        if logger:
            logger.error(f"Service {client.srv_name} is not available after 1 second wait")
        return None

    future = client.call_async(request)
    start_time = time.time()
    while not future.done():
        if time.time() - start_time > timeout:
            return None
        time.sleep(0.01)

    return future.result()

@auton_bp.route('/enable_auton/', methods=['POST'])
def enable_auton():
    try:
        node = get_node()
        enabled = request.json.get('enabled', False)
        waypoints = request.json.get('waypoints', [])

        if not isinstance(waypoints, list):
            return jsonify({'status': 'error', 'message': 'waypoints must be a list'}), 400

        enable_auton_srv = node.create_client(EnableAuton, "/enable_auton")

        auton_request = EnableAuton.Request(
            enable=enabled,
            waypoints=[
                GPSWaypoint(
                    tag_id=wp.get("tag_id", -1),
                    latitude_degrees=wp.get("latitude_degrees", 0.0),
                    longitude_degrees=wp.get("longitude_degrees", 0.0),
                    type=WaypointType(val=int(wp.get("type", 0))),
                    enable_costmap=wp.get("enable_costmap", True),
                )
                for wp in waypoints
            ],
        )

        node.get_logger().info(f"Calling ROS service /enable_auton with {len(waypoints)} waypoints")

        result = _call_service_sync(enable_auton_srv, auton_request, logger=node.get_logger())
        if result is None:
            return jsonify({'status': 'error', 'message': 'Service /enable_auton is not available or timed out'}), 500

        return jsonify({
            'status': 'success',
            'enabled': enabled,
            'waypoint_count': len(waypoints)
        })

    except Exception as e:
        error_details = traceback.format_exc()
        try:
            node = get_node()
            node.get_logger().error(f"Error in enable_auton: {str(e)}\\n{error_details}")
        except:
            pass
        return jsonify({'status': 'error', 'message': str(e)}), 500

@auton_bp.route('/teleop/enable/', methods=['POST'])
def enable_teleop():
    try:
        enabled = request.json.get('enabled', False)

        if not isinstance(enabled, bool):
            return jsonify({'status': 'error', 'message': 'enabled must be boolean'}), 400

        node = get_node()
        enable_teleop_srv = node.create_client(SetBool, "/enable_teleop")

        teleop_request = SetBool.Request()
        teleop_request.data = enabled

        result = _call_service_sync(enable_teleop_srv, teleop_request, logger=node.get_logger())
        if result is None:
            return jsonify({'status': 'error', 'message': 'Service /enable_teleop is not available or timed out'}), 500

        return jsonify({'status': 'success', 'enabled': enabled})

    except Exception as e:
        error_details = traceback.format_exc()
        node = get_node()
        node.get_logger().error(f"Error in enable_teleop: {error_details}")
        return jsonify({'status': 'error', 'message': str(e)}), 500
