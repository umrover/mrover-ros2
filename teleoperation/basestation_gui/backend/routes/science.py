from fastapi import APIRouter, HTTPException

from backend.managers.ros import get_node, get_logger, get_service_client
from backend.models_pydantic import ServoPositionCommand
from backend.utils.ros_service import call_service_async
from mrover.srv import ServoPosition

router = APIRouter(prefix="/api", tags=["science"])


@router.post("/science/funnel-servo/position/")
async def funnel_servo_position(data: ServoPositionCommand):
    try:
        node = get_node()
        client = get_service_client(ServoPosition, "/sp_funnel_servo")

        request = ServoPosition.Request()
        request.header.stamp = node.get_clock().now().to_msg()
        request.header.frame_id = ""
        request.names = ["funnel"]
        request.positions = [data.position]

        result = await call_service_async(client, request)
        if result is None:
            raise HTTPException(status_code=500, detail="Service call timed out")

        at_tgt = result.at_tgts[0] if result.at_tgts else False
        return {'status': 'success', 'position': data.position, 'at_tgt': at_tgt}

    except HTTPException:
        raise
    except Exception as e:
        get_logger().error(f"funnel_servo_position error: {e}")
        raise HTTPException(status_code=500, detail=str(e))
