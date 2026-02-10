"""Waypoints REST API Views"""

from django.views.decorators.csrf import csrf_exempt
from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from backend.waypoints import (
    get_basic_waypoint_list,
    save_basic_waypoint_list,
    get_auton_waypoint_list,
    save_auton_waypoint_list,
    get_current_basic_course,
    save_current_basic_course,
    get_current_auton_course,
    save_current_auton_course,
    delete_auton_waypoint_from_course,
)


@api_view(['GET'])
def basic_waypoints_list(request):
    """Get list of all basic waypoints"""
    try:
        waypoints = get_basic_waypoint_list()
        return Response({'status': 'success', 'waypoints': waypoints})
    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@csrf_exempt
@api_view(['POST'])
def basic_waypoints_save(request):
    """Save basic waypoints list"""
    try:
        waypoints = request.data.get('waypoints', [])
        if not isinstance(waypoints, list):
            return Response({'status': 'error', 'message': 'waypoints must be a list'},
                           status=status.HTTP_400_BAD_REQUEST)

        save_basic_waypoint_list(waypoints)
        return Response({'status': 'success', 'count': len(waypoints)})
    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@api_view(['GET'])
def auton_waypoints_list(request):
    """Get list of all auton waypoints"""
    try:
        waypoints = get_auton_waypoint_list()
        return Response({'status': 'success', 'waypoints': waypoints})
    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@csrf_exempt
@api_view(['POST'])
def auton_waypoints_save(request):
    """Save auton waypoints list"""
    try:
        waypoints = request.data.get('waypoints', [])
        if not isinstance(waypoints, list):
            return Response({'status': 'error', 'message': 'waypoints must be a list'},
                           status=status.HTTP_400_BAD_REQUEST)

        save_auton_waypoint_list(waypoints)
        return Response({'status': 'success', 'count': len(waypoints)})
    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@api_view(['GET'])
def basic_course_current(request):
    """Get current basic course"""
    try:
        course = get_current_basic_course()
        return Response({'status': 'success', 'course': course})
    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@csrf_exempt
@api_view(['POST'])
def basic_course_save(request):
    """Save current basic course"""
    try:
        course = request.data.get('course', [])
        if not isinstance(course, list):
            return Response({'status': 'error', 'message': 'course must be a list'},
                           status=status.HTTP_400_BAD_REQUEST)

        save_current_basic_course(course)
        return Response({'status': 'success'})
    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@api_view(['GET'])
def auton_course_current(request):
    """Get current auton course"""
    try:
        course = get_current_auton_course()
        return Response({'status': 'success', 'course': course})
    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@csrf_exempt
@api_view(['POST'])
def auton_course_save(request):
    """Save current auton course"""
    try:
        course = request.data.get('course', [])
        if not isinstance(course, list):
            return Response({'status': 'error', 'message': 'course must be a list'},
                           status=status.HTTP_400_BAD_REQUEST)

        save_current_auton_course(course)
        return Response({'status': 'success'})
    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@csrf_exempt
@api_view(['DELETE'])
def auton_waypoint_delete(request, waypoint_id):
    """Delete waypoint from auton course"""
    try:
        waypoint = request.data
        if not waypoint:
            return Response({'status': 'error', 'message': 'waypoint data required'},
                           status=status.HTTP_400_BAD_REQUEST)

        delete_auton_waypoint_from_course(waypoint)
        return Response({'status': 'success', 'deleted': waypoint_id})
    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)
