from fastapi import APIRouter, HTTPException
from backend.ros_manager import get_node
from backend.models_pydantic import GimbalAdjustRequest, ServoPositionRequest
from mrover.srv import PanoramaStart, PanoramaEnd, ServoSetPos, ServoPosition
import numpy as np
import cv2
import time
import math

router = APIRouter(prefix="/api", tags=["chassis"])

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

@router.post("/panorama/start/")
def panorama_start():
    try:
        node = get_node()
        pano_start_srv = node.create_client(PanoramaStart, "/panorama/start")

        pano_request = PanoramaStart.Request()
        result = _call_service_sync(pano_start_srv, pano_request)

        if result is None:
            raise HTTPException(status_code=500, detail="Service call failed")

        return {'status': 'success'}

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/panorama/stop/")
def panorama_stop():
    try:
        node = get_node()
        pano_end_srv = node.create_client(PanoramaEnd, "/panorama/end")

        pano_request = PanoramaEnd.Request()
        result = _call_service_sync(pano_end_srv, pano_request, timeout=30.0)

        if result is None:
            raise HTTPException(status_code=500, detail="Service call failed")

        if not result.success:
            raise HTTPException(status_code=500, detail="Panorama failed")

        img_msg = result.img
        img_np = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
            img_msg.height, img_msg.width, -1
        )

        if img_np.shape[2] == 4:
            img_np = cv2.cvtColor(img_np, cv2.COLOR_RGBA2BGR)

        timestamp = node.get_clock().now().nanoseconds
        filename = f"../../data/{timestamp}_panorama.png"
        cv2.imwrite(filename, img_np)

        return {
            'status': 'success',
            'image_path': filename,
            'timestamp': timestamp
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/gimbal/adjust/")
def gimbal_adjust(data: GimbalAdjustRequest):
    try:
        if data.joint not in ['pitch', 'yaw']:
            raise HTTPException(status_code=400, detail="Joint must be pitch or yaw")

        adjustment_rad = math.radians(data.adjustment)

        node = get_node()
        gimbal_srv = node.create_client(ServoSetPos, "/gimbal_set_position")

        gimbal_request = ServoSetPos.Request()
        gimbal_request.position = adjustment_rad
        gimbal_request.is_counterclockwise = adjustment_rad < 0 if not data.absolute else False

        result = _call_service_sync(gimbal_srv, gimbal_request)

        if result is None:
            raise HTTPException(status_code=500, detail="Service call failed")

        if not result.success:
            raise HTTPException(status_code=500, detail="Gimbal adjustment failed")

        return {
            'status': 'success',
            'joint': data.joint,
            'adjustment': data.adjustment
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/sp_funnel_servo/")
def sp_funnel_servo(data: ServoPositionRequest):
    try:
        node = get_node()
        servo_srv = node.create_client(ServoPosition, "/sp_funnel_servo")

        servo_request = ServoPosition.Request()
        servo_request.header.stamp = node.get_clock().now().to_msg()
        servo_request.header.frame_id = ""
        servo_request.name = data.name
        servo_request.position = data.position

        result = _call_service_sync(servo_srv, servo_request)

        if result is None:
            raise HTTPException(status_code=500, detail="Service call failed")

        return {
            'status': 'success',
            'at_tgt': result.at_tgt
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))