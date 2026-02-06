"""Science REST API Views"""
from django.views.decorators.csrf import csrf_exempt

from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from backend.consumers.ros_manager import get_node
from mrover.srv import EnableBool, ServoSetPos
import time

HEATER_NAMES = ["a0", "a1", "b0", "b1"]


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
def heater_control(request, heater_name):
    """Enable/disable heater (a0, a1, b0, b1)"""
    try:
        if heater_name not in HEATER_NAMES:
            return Response({'status': 'error', 'message': f'Invalid heater: {heater_name}'},
                           status=status.HTTP_404_NOT_FOUND)

        enable = request.data.get('enable', False)
        if not isinstance(enable, bool):
            return Response({'status': 'error', 'message': 'enable must be boolean'},
                           status=status.HTTP_400_BAD_REQUEST)

        node = get_node()
        heater_srv = node.create_client(EnableBool, f"/science_enable_heater_{heater_name}")

        heater_request = EnableBool.Request()
        heater_request.enable = enable

        result = _call_service_sync(heater_srv, heater_request)
        if result is None:
            return Response({'status': 'error', 'message': 'Service call failed'},
                           status=status.HTTP_500_INTERNAL_SERVER_ERROR)

        return Response({'status': 'success', 'heater': heater_name, 'enabled': enable})

    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


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


@csrf_exempt
@api_view(['POST'])
def auto_shutoff(request):
    """Enable/disable heater auto shutoff"""
    try:
        enable = request.data.get('enable', False)
        if not isinstance(enable, bool):
            return Response({'status': 'error', 'message': 'enable must be boolean'},
                           status=status.HTTP_400_BAD_REQUEST)

        node = get_node()
        auto_shutoff_srv = node.create_client(EnableBool, "/science_change_heater_auto_shutoff_state")

        shutoff_request = EnableBool.Request()
        shutoff_request.enable = enable

        result = _call_service_sync(auto_shutoff_srv, shutoff_request)
        if result is None:
            return Response({'status': 'error', 'message': 'Service call failed'},
                           status=status.HTTP_500_INTERNAL_SERVER_ERROR)

        return Response({'status': 'success', 'auto_shutoff_enabled': enable})

    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@csrf_exempt
@api_view(['POST'])
def white_leds(request, site):
    """Control white LEDs for site (a or b)"""
    try:
        if site in ["a", "A", "0"]:
            site_index = 0
            site_name = "a"
        elif site in ["b", "B", "1"]:
            site_index = 1
            site_name = "b"
        else:
            return Response({'status': 'error', 'message': 'Invalid site'},
                           status=status.HTTP_404_NOT_FOUND)

        enable = request.data.get('enable', False)
        if not isinstance(enable, bool):
            return Response({'status': 'error', 'message': 'enable must be boolean'},
                           status=status.HTTP_400_BAD_REQUEST)

        node = get_node()
        white_led_srv = node.create_client(EnableBool, f"/science_enable_white_led_{site_name}")

        led_request = EnableBool.Request()
        led_request.enable = enable

        result = _call_service_sync(white_led_srv, led_request)
        if result is None:
            return Response({'status': 'error', 'message': 'Service call failed'},
                           status=status.HTTP_500_INTERNAL_SERVER_ERROR)

        return Response({'status': 'success', 'site': site_name, 'enabled': enable})

    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@csrf_exempt
@api_view(['POST'])
def limit_switch(request):
    """Enable/disable limit switch sensor actuator"""
    try:
        enable = request.data.get('enable', False)
        if not isinstance(enable, bool):
            return Response({'status': 'error', 'message': 'enable must be boolean'},
                           status=status.HTTP_400_BAD_REQUEST)

        node = get_node()
        limit_switch_srv = node.create_client(EnableBool, "/sa_enable_limit_switch_sensor_actuator")

        switch_request = EnableBool.Request()
        switch_request.enable = enable

        result = _call_service_sync(limit_switch_srv, switch_request)
        if result is None:
            return Response({'status': 'error', 'message': 'Service call failed'},
                           status=status.HTTP_500_INTERNAL_SERVER_ERROR)

        return Response({'status': 'success', 'limit_switch_enabled': enable})

    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)
