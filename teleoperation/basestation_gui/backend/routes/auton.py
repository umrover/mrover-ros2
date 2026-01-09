from fastapi import APIRouter, HTTPException
from backend.ros_manager import get_node
from backend.models_pydantic import AutonEnableRequest, TeleopEnableRequest
from backend.led_manager import set_teleop_enabled
from mrover.srv import EnableAuton
from mrover.msg import GPSWaypoint, WaypointType
from std_srvs.srv import SetBool
import time
import traceback

router = APIRouter(prefix="/api", tags=["auton"])

def _call_service_sync(client, request, timeout=5.0, logger=None):
    if not client.wait_for_service(timeout_sec=1.0):
        if logger:
            logger.error(f"Service {client.srv_name} is not available after 1 second wait")
        return None

    future = client.call_async(request)
    start_time = time.time()
    while not future.done():
        if time.time() - start_time > timeout:
            return None
        time.sleep(0.01)

    return future.result()

@router.post("/enable_auton/")
def enable_auton(data: AutonEnableRequest):
    try:
        node = get_node()

        enable_auton_srv = node.create_client(EnableAuton, "/enable_auton")

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

        result = _call_service_sync(enable_auton_srv, auton_request, logger=node.get_logger())
        if result is None:
            raise HTTPException(status_code=500, detail="Service /enable_auton is not available or timed out")

        return {
            'status': 'success',
            'enabled': data.enabled,
            'waypoint_count': len(data.waypoints)
        }

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