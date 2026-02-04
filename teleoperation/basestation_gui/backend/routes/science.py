import asyncio

from fastapi import APIRouter, HTTPException

from backend.managers.ros import get_node, get_logger, get_service_client
from backend.models_pydantic import GearDiffRequest
from mrover.srv import ServoPosition

router = APIRouter(prefix="/api", tags=["science"])


async def call_service_async(client, request, timeout=5.0):
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


@router.post("/science/gear-diff/position/")
async def gear_diff_position(data: GearDiffRequest):
    try:
        node = get_node()
        client = get_service_client(ServoPosition, "/sp_funnel_servo")

        sp_funnel_request = ServoPosition.Request()
        sp_funnel_request.header.stamp = node.get_clock().now().to_msg()
        sp_funnel_request.header.frame_id = ""
        sp_funnel_request.names = ["funnel"]
        sp_funnel_request.positions = [data.position]

        result = await call_service_async(client, sp_funnel_request)
        if result is None:
            raise HTTPException(status_code=500, detail="Service call failed")

        at_tgt = result.at_tgts[0] if result.at_tgts else False
        return {'status': 'success', 'position': data.position, 'at_tgt': at_tgt}

    except HTTPException:
        raise
    except Exception as e:
        get_logger().error(f"gear_diff_position error: {e}")
        raise HTTPException(status_code=500, detail=str(e))