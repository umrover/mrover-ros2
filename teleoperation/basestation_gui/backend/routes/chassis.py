import asyncio

from fastapi import APIRouter, HTTPException
import numpy as np
import cv2

from backend.managers.ros import get_node, get_service_client
from backend.models_pydantic import GimbalAdjustRequest, ServoPositionRequest
from mrover.srv import PanoramaStart, PanoramaEnd, ServoPosition

router = APIRouter(prefix="/api/chassis", tags=["chassis"])


async def call_service_async(client, request, timeout=10.0):
    if not client.wait_for_service(timeout_sec=1.0):
        return None

    future = client.call_async(request)
    try:
        await asyncio.wait_for(
            asyncio.get_event_loop().run_in_executor(None, future.result),
            timeout=timeout
        )
        return future.result()
    except asyncio.TimeoutError:
        return None


@router.post("/panorama/start/")
async def panorama_start():
    try:
        client = get_service_client(PanoramaStart, "/panorama/start")
        pano_request = PanoramaStart.Request()
        result = await call_service_async(client, pano_request)

        if result is None:
            raise HTTPException(status_code=500, detail="Service call failed")

        return {'status': 'success'}

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/panorama/stop/")
async def panorama_stop():
    try:
        client = get_service_client(PanoramaEnd, "/panorama/end")
        pano_request = PanoramaEnd.Request()
        result = await call_service_async(client, pano_request, timeout=30.0)

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

        node = get_node()
        timestamp = node.get_clock().now().nanoseconds
        filename = f"../../data/{timestamp}_panorama.png"
        cv2.imwrite(filename, img_np)

        return {
            'status': 'success',
            'image_path': filename,
            'timestamp': timestamp
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/gimbal/adjust/")
async def gimbal_adjust(data: GimbalAdjustRequest):
    try:
        if data.joint not in ['pitch', 'yaw']:
            raise HTTPException(status_code=400, detail="Joint must be pitch or yaw")

        node = get_node()
        client = get_service_client(ServoPosition, "/gimbal_servo")

        gimbal_request = ServoPosition.Request()
        gimbal_request.header.stamp = node.get_clock().now().to_msg()
        gimbal_request.header.frame_id = ""
        gimbal_request.names = [data.joint]
        gimbal_request.positions = [data.adjustment]

        result = await call_service_async(client, gimbal_request)

        if result is None:
            raise HTTPException(status_code=500, detail="Service call failed")

        if not result.at_tgts or not result.at_tgts[0]:
            raise HTTPException(status_code=500, detail="Gimbal adjustment failed")

        return {
            'status': 'success',
            'joint': data.joint,
            'adjustment': data.adjustment
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/sp_funnel_servo/")
async def sp_funnel_servo(data: ServoPositionRequest):
    try:
        node = get_node()
        client = get_service_client(ServoPosition, "/sp_funnel_servo")

        servo_request = ServoPosition.Request()
        servo_request.header.stamp = node.get_clock().now().to_msg()
        servo_request.header.frame_id = ""
        servo_request.names = data.names
        servo_request.positions = data.positions

        result = await call_service_async(client, servo_request)

        if result is None:
            raise HTTPException(status_code=500, detail="Service call failed")

        return {
            'status': 'success',
            'at_tgts': result.at_tgts
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))