"""Science REST API Views"""
from django.views.decorators.csrf import csrf_exempt

from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from backend.consumers.ros_manager import get_node
from mrover.srv import ServoSetPos
import time


def _call_service_sync(client, request, timeout=5.0):
    """Call ROS service synchronously with timeout"""
    if not client.wait_for_service(timeout_sec=1.0):
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
def gear_diff_position(request):
    """Set gear differential position"""
    try:
        position = request.data.get('position')
        is_ccw = request.data.get('is_counterclockwise', False)

        if position is None:
            return Response({'status': 'error', 'message': 'position required'},
                           status=status.HTTP_400_BAD_REQUEST)

        node = get_node()
        gear_diff_srv = node.create_client(ServoSetPos, "/sa_gear_diff_set_position")

        gear_diff_request = ServoSetPos.Request()
        gear_diff_request.position = float(position)
        gear_diff_request.is_counterclockwise = is_ccw

        result = _call_service_sync(gear_diff_srv, gear_diff_request)
        if result is None:
            return Response({'status': 'error', 'message': 'Service call failed'},
                           status=status.HTTP_500_INTERNAL_SERVER_ERROR)

        return Response({'status': 'success', 'position': position, 'is_counterclockwise': is_ccw})

    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


