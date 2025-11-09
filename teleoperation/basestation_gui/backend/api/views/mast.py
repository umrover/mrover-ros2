"""Mast REST API Views"""
from django.views.decorators.csrf import csrf_exempt

from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from backend.consumers.ros_manager import get_node
from mrover.srv import PanoramaStart, PanoramaEnd, ServoSetPos
import numpy as np
import cv2
import time
import math


def _call_service_sync(client, request, timeout=10.0):
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
def panorama_start(request):
    """Start panorama capture"""
    try:
        node = get_node()
        pano_start_srv = node.create_client(PanoramaStart, "/panorama/start")

        pano_request = PanoramaStart.Request()
        result = _call_service_sync(pano_start_srv, pano_request)

        if result is None:
            return Response({'status': 'error', 'message': 'Service call failed'},
                           status=status.HTTP_500_INTERNAL_SERVER_ERROR)

        return Response({'status': 'success'})

    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@csrf_exempt
@api_view(['POST'])
def panorama_stop(request):
    """Stop panorama capture and save image"""
    try:
        node = get_node()
        pano_end_srv = node.create_client(PanoramaEnd, "/panorama/end")

        pano_request = PanoramaEnd.Request()
        result = _call_service_sync(pano_end_srv, pano_request, timeout=30.0)

        if result is None:
            return Response({'status': 'error', 'message': 'Service call failed'},
                           status=status.HTTP_500_INTERNAL_SERVER_ERROR)

        if not result.success:
            return Response({'status': 'error', 'message': 'Panorama failed'},
                           status=status.HTTP_500_INTERNAL_SERVER_ERROR)

        # Process and save image
        img_msg = result.img
        img_np = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
            img_msg.height, img_msg.width, -1
        )

        if img_np.shape[2] == 4:
            img_np = cv2.cvtColor(img_np, cv2.COLOR_RGBA2BGR)

        timestamp = node.get_clock().now().nanoseconds
        filename = f"../../data/{timestamp}_panorama.png"
        cv2.imwrite(filename, img_np)

        return Response({
            'status': 'success',
            'image_path': filename,
            'timestamp': timestamp
        })

    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@csrf_exempt
@api_view(['POST'])
def gimbal_adjust(request):
    """Adjust gimbal position by delta or set absolute position"""
    try:
        joint = request.data.get('joint')
        adjustment = request.data.get('adjustment')
        absolute = request.data.get('absolute', False)

        if not joint or adjustment is None:
            return Response({'status': 'error', 'message': 'Missing joint or adjustment parameter'},
                           status=status.HTTP_400_BAD_REQUEST)

        if joint not in ['pitch', 'yaw']:
            return Response({'status': 'error', 'message': 'Joint must be pitch or yaw'},
                           status=status.HTTP_400_BAD_REQUEST)

        adjustment_rad = math.radians(float(adjustment))

        node = get_node()
        gimbal_srv = node.create_client(ServoSetPos, "/gimbal_set_position")

        gimbal_request = ServoSetPos.Request()
        gimbal_request.position = adjustment_rad
        gimbal_request.is_counterclockwise = adjustment_rad < 0 if not absolute else False

        result = _call_service_sync(gimbal_srv, gimbal_request)

        if result is None:
            return Response({'status': 'error', 'message': 'Service call failed'},
                           status=status.HTTP_500_INTERNAL_SERVER_ERROR)

        if not result.success:
            return Response({'status': 'error', 'message': 'Gimbal adjustment failed'},
                           status=status.HTTP_500_INTERNAL_SERVER_ERROR)

        return Response({
            'status': 'success',
            'joint': joint,
            'adjustment': adjustment
        })

    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)
