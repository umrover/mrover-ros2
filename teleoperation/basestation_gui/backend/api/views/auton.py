"""Auton REST API Views"""

from django.views.decorators.csrf import csrf_exempt
from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from backend.consumers.ros_manager import get_node
from mrover.srv import EnableAuton
from mrover.msg import GPSWaypoint, WaypointType
from std_srvs.srv import SetBool
import time
import traceback


def _call_service_sync(client, request, timeout=5.0, logger=None):
    """Call ROS service synchronously with timeout"""
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


@csrf_exempt
@api_view(['POST'])
def enable_auton(request):
    """Enable/disable autonomous navigation with waypoints"""
    try:
        node = get_node()
        enabled = request.data.get('enabled', False)
        waypoints = request.data.get('waypoints', [])

        if not isinstance(waypoints, list):
            return Response({'status': 'error', 'message': 'waypoints must be a list'},
                           status=status.HTTP_400_BAD_REQUEST)

        enable_auton_srv = node.create_client(EnableAuton, "/enable_auton")

        auton_request = EnableAuton.Request(
            enable=enabled,
            waypoints=[
                GPSWaypoint(
                    tag_id=wp.get("tag_id", -1),
                    latitude_degrees=wp.get("latitude_degrees", 0.0),
                    longitude_degrees=wp.get("longitude_degrees", 0.0),
                    type=WaypointType(val=int(wp.get("type", 0))),
                    enable_costmap=wp.get("enable_costmap", True),
                )
                for wp in waypoints
            ],
        )

        node.get_logger().info(f"Calling ROS service /enable_auton with {len(waypoints)} waypoints")

        result = _call_service_sync(enable_auton_srv, auton_request, logger=node.get_logger())
        if result is None:
            return Response({'status': 'error', 'message': 'Service /enable_auton is not available or timed out'},
                           status=status.HTTP_500_INTERNAL_SERVER_ERROR)

        return Response({
            'status': 'success',
            'enabled': enabled,
            'waypoint_count': len(waypoints)
        })

    except Exception as e:
        error_details = traceback.format_exc()
        try:
            node = get_node()
            node.get_logger().error(f"Error in enable_auton: {str(e)}\n{error_details}")
        except:
            pass
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@csrf_exempt
@api_view(['POST'])
def enable_teleop(request):
    """Enable/disable teleoperation mode"""
    try:
        enabled = request.data.get('enabled', False)

        if not isinstance(enabled, bool):
            return Response({'status': 'error', 'message': 'enabled must be boolean'},
                           status=status.HTTP_400_BAD_REQUEST)

        node = get_node()
        enable_teleop_srv = node.create_client(SetBool, "/enable_teleop")

        teleop_request = SetBool.Request()
        teleop_request.data = enabled

        result = _call_service_sync(enable_teleop_srv, teleop_request, logger=node.get_logger())
        if result is None:
            return Response({'status': 'error', 'message': 'Service /enable_teleop is not available or timed out'},
                           status=status.HTTP_500_INTERNAL_SERVER_ERROR)

        return Response({'status': 'success', 'enabled': enabled})

    except Exception as e:
        error_details = traceback.format_exc()
        node = get_node()
        node.get_logger().error(f"Error in enable_teleop: {error_details}")
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)
