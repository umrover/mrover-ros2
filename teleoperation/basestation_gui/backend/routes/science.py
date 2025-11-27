from flask import Blueprint, jsonify, request
from backend.consumers.ros_manager import get_node
from mrover.srv import ServoSetPos
import time

science_bp = Blueprint('science', __name__)

def _call_service_sync(client, request, timeout=5.0):
    if not client.wait_for_service(timeout_sec=1.0):
        return None

    future = client.call_async(request)
    start_time = time.time()
    while not future.done():
        if time.time() - start_time > timeout:
            return None
        time.sleep(0.01)

    return future.result()

@science_bp.route('/gear_diff_position/', methods=['POST'])
def gear_diff_position():
    try:
        position = request.json.get('position')
        is_ccw = request.json.get('is_counterclockwise', False)

        if position is None:
            return jsonify({'status': 'error', 'message': 'position required'}), 400

        node = get_node()
        sp_funnel_srv = node.create_client(ServoSetPos, "/sp_funnel_set_position")

        sp_funnel_request = ServoSetPos.Request()
        sp_funnel_request.position = float(position)
        sp_funnel_request.is_counterclockwise = is_ccw

        result = _call_service_sync(sp_funnel_srv, sp_funnel_request)
        if result is None:
            return jsonify({'status': 'error', 'message': 'Service call failed'}), 500

        return jsonify({'status': 'success', 'position': position, 'is_counterclockwise': is_ccw})

    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500
