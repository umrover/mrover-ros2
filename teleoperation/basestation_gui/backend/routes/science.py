from fastapi import APIRouter, HTTPException
from backend.ros_manager import get_node
from backend.models_pydantic import GearDiffRequest
from mrover.srv import ServoPosition
import time

router = APIRouter(prefix="/api", tags=["science"])

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

@router.post("/gear_diff_position/")
def gear_diff_position(data: GearDiffRequest):
    try:
        node = get_node()
        sp_funnel_srv = node.create_client(ServoPosition, "/sp_funnel_servo")

        sp_funnel_request = ServoPosition.Request()
        sp_funnel_request.header.stamp = node.get_clock().now().to_msg()
        sp_funnel_request.header.frame_id = ""
        sp_funnel_request.name = ["funnel"]
        sp_funnel_request.position = [data.position]

        result = _call_service_sync(sp_funnel_srv, sp_funnel_request)
        if result is None:
            raise HTTPException(status_code=500, detail="Service call failed")

        return {'status': 'success', 'position': data.position, 'at_tgt': result.at_tgt[0]}

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))