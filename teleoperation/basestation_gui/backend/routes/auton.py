import asyncio
import traceback

from fastapi import APIRouter, HTTPException

from backend.managers.ros import get_node, get_service_client
from backend.models_pydantic import AutonEnableRequest, TeleopEnableRequest
from backend.managers.led import set_teleop_enabled
from mrover.srv import EnableAuton
from mrover.msg import GPSWaypoint, WaypointType

router = APIRouter(prefix="/api", tags=["auton"])


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


@router.post("/enable_auton/")
async def enable_auton(data: AutonEnableRequest):
    try:
        client = get_service_client(EnableAuton, "/enable_auton")

        auton_request = EnableAuton.Request(
            enable=data.enabled,
            waypoints=[
                GPSWaypoint(
                    tag_id=wp.tag_id,
                    latitude_degrees=wp.latitude_degrees,
                    longitude_degrees=wp.longitude_degrees,
                    type=WaypointType(val=int(wp.type)),
                    enable_costmap=wp.enable_costmap,
                    coverage_radius=wp.coverage_radius,
                )
                for wp in data.waypoints
            ],
        )

        result = await call_service_async(client, auton_request)
        if result is None:
            raise HTTPException(status_code=500, detail="Service /enable_auton is not available or timed out")

        return {
            'status': 'success',
            'enabled': data.enabled,
            'waypoint_count': len(data.waypoints)
        }

    except HTTPException:
        raise
    except Exception as e:
        error_details = traceback.format_exc()
        try:
            node = get_node()
            node.get_logger().error(f"Error in enable_auton: {str(e)}\n{error_details}")
        except:
            pass
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/enable_teleop/")
def enable_teleop(data: TeleopEnableRequest):
    set_teleop_enabled(data.enabled)
    return {'status': 'success', 'enabled': data.enabled}