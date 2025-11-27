from fastapi import APIRouter, HTTPException
from backend.ros_manager import get_node
from backend.models_pydantic import GearDiffRequest
from mrover.srv import ServoSetPos
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
        sp_funnel_srv = node.create_client(ServoSetPos, "/sp_funnel_set_position")

        sp_funnel_request = ServoSetPos.Request()
        sp_funnel_request.position = data.position
        sp_funnel_request.is_counterclockwise = data.is_counterclockwise

        result = _call_service_sync(sp_funnel_srv, sp_funnel_request)
        if result is None:
            raise HTTPException(status_code=500, detail="Service call failed")

        return {'status': 'success', 'position': data.position, 'is_counterclockwise': data.is_counterclockwise}

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))