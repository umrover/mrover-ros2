from flask import Blueprint, jsonify, request
from backend.consumers.ros_manager import get_node
from mrover.srv import PanoramaStart, PanoramaEnd, ServoSetPos
import numpy as np
import cv2
import time
import math

mast_bp = Blueprint('mast', __name__)

def _call_service_sync(client, request, timeout=10.0):
    if not client.wait_for_service(timeout_sec=1.0):
        return None

    future = client.call_async(request)
    start_time = time.time()
    while not future.done():
        if time.time() - start_time > timeout:
            return None
        time.sleep(0.01)

    return future.result()

@mast_bp.route('/panorama/start/', methods=['POST'])
def panorama_start():
    try:
        node = get_node()
        pano_start_srv = node.create_client(PanoramaStart, "/panorama/start")

        pano_request = PanoramaStart.Request()
        result = _call_service_sync(pano_start_srv, pano_request)

        if result is None:
            return jsonify({'status': 'error', 'message': 'Service call failed'}), 500

        return jsonify({'status': 'success'})

    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@mast_bp.route('/panorama/stop/', methods=['POST'])
def panorama_stop():
    try:
        node = get_node()
        pano_end_srv = node.create_client(PanoramaEnd, "/panorama/end")

        pano_request = PanoramaEnd.Request()
        result = _call_service_sync(pano_end_srv, pano_request, timeout=30.0)

        if result is None:
            return jsonify({'status': 'error', 'message': 'Service call failed'}), 500

        if not result.success:
            return jsonify({'status': 'error', 'message': 'Panorama failed'}), 500

        img_msg = result.img
        img_np = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
            img_msg.height, img_msg.width, -1
        )

        if img_np.shape[2] == 4:
            img_np = cv2.cvtColor(img_np, cv2.COLOR_RGBA2BGR)

        timestamp = node.get_clock().now().nanoseconds
        filename = f"../../data/{timestamp}_panorama.png"
        cv2.imwrite(filename, img_np)

        return jsonify({
            'status': 'success',
            'image_path': filename,
            'timestamp': timestamp
        })

    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@mast_bp.route('/gimbal/adjust/', methods=['POST'])
def gimbal_adjust():
    try:
        joint = request.json.get('joint')
        adjustment = request.json.get('adjustment')
        absolute = request.json.get('absolute', False)

        if not joint or adjustment is None:
            return jsonify({'status': 'error', 'message': 'Missing joint or adjustment parameter'}), 400

        if joint not in ['pitch', 'yaw']:
            return jsonify({'status': 'error', 'message': 'Joint must be pitch or yaw'}), 400

        adjustment_rad = math.radians(float(adjustment))

        node = get_node()
        gimbal_srv = node.create_client(ServoSetPos, "/gimbal_set_position")

        gimbal_request = ServoSetPos.Request()
        gimbal_request.position = adjustment_rad
        gimbal_request.is_counterclockwise = adjustment_rad < 0 if not absolute else False

        result = _call_service_sync(gimbal_srv, gimbal_request)

        if result is None:
            return jsonify({'status': 'error', 'message': 'Service call failed'}), 500

        if not result.success:
            return jsonify({'status': 'error', 'message': 'Gimbal adjustment failed'}), 500

        return jsonify({
            'status': 'success',
            'joint': joint,
            'adjustment': adjustment
        })

    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500
