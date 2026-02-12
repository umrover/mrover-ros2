import traceback

from fastapi import APIRouter, HTTPException

from backend.managers.ros import get_node, get_service_client
from backend.models_pydantic import AutonEnableRequest, TeleopEnableRequest
from backend.managers.led import set_teleop_enabled
from backend.utils.ros_service import call_service_async
from mrover.srv import EnableAuton
from mrover.msg import GPSWaypoint, WaypointType
from std_srvs.srv import SetBool

router = APIRouter(prefix="/api", tags=["auton"])


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


@router.post("/toggle_pure_pursuit/")
async def toggle_pure_pursuit(data: TeleopEnableRequest):
    client = get_service_client(SetBool, "/toggle_pure_pursuit")
    request = SetBool.Request(data=data.enabled)
    result = await call_service_async(client, request)
    if result is None:
        raise HTTPException(status_code=500, detail="Service /toggle_pure_pursuit is not available or timed out")
    return {'status': 'success', 'enabled': data.enabled}


@router.post("/toggle_path_relaxation/")
async def toggle_path_relaxation(data: TeleopEnableRequest):
    client = get_service_client(SetBool, "/toggle_path_relaxation")
    request = SetBool.Request(data=data.enabled)
    result = await call_service_async(client, request)
    if result is None:
        raise HTTPException(status_code=500, detail="Service /toggle_path_relaxation is not available or timed out")
    return {'status': 'success', 'enabled': data.enabled}


@router.post("/toggle_path_interpolation/")
async def toggle_path_interpolation(data: TeleopEnableRequest):
    client = get_service_client(SetBool, "/toggle_path_interpolation")
    request = SetBool.Request(data=data.enabled)
    result = await call_service_async(client, request)
    if result is None:
        raise HTTPException(status_code=500, detail="Service /toggle_path_interpolation is not available or timed out")
    return {'status': 'success', 'enabled': data.enabled}


# TODO(object): Implement the stereo detector toggle endpoint.
# This should:
#   1. Get a service client: get_service_client(SetBool, "/toggle_stereo_detector")
#   2. Create the request: request = SetBool.Request(data=data.enabled)
#   3. Call: result = await call_service_async(client, request)
#   4. Handle None result (service unavailable) with HTTPException 500
#   5. Return {'status': 'success', 'enabled': data.enabled}
# Look at toggle_pure_pursuit() above for the exact pattern.
@router.post("/toggle_stereo_detector/")
async def toggle_stereo_detector(data: TeleopEnableRequest):
    raise HTTPException(status_code=501, detail="Not implemented yet")


# TODO(object): Same as above, but call "/toggle_image_detector" service.
@router.post("/toggle_image_detector/")
async def toggle_image_detector(data: TeleopEnableRequest):
    raise HTTPException(status_code=501, detail="Not implemented yet")