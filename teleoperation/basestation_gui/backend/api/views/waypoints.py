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
    create_recording,
    add_waypoint_to_recording,
    get_all_recordings,
    get_recording_waypoints,
    delete_recording,
    clear_all_basic_waypoints,
    clear_all_recordings,
)
from backend.recording_manager import get_recording_manager


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


@csrf_exempt
@api_view(['POST'])
def recording_create(request):
    try:
        name = request.data.get('name')
        is_drone = request.data.get('is_drone', False)

        if not name:
            return Response({'status': 'error', 'message': 'name is required'},
                           status=status.HTTP_400_BAD_REQUEST)

        manager = get_recording_manager()
        recording_id = manager.start_recording(name, is_drone)
        return Response({'status': 'success', 'recording_id': recording_id})
    except ValueError as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_400_BAD_REQUEST)
    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@csrf_exempt
@api_view(['POST'])
def recording_stop(request):
    try:
        manager = get_recording_manager()
        result = manager.stop_recording()
        return Response({
            'status': 'success',
            'recording_id': result['recording_id'],
            'waypoint_count': result['waypoint_count']
        })
    except ValueError as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_400_BAD_REQUEST)
    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@csrf_exempt
@api_view(['POST'])
def recording_add_waypoint(request, recording_id):
    try:
        lat = request.data.get('lat')
        lon = request.data.get('lon')
        sequence = request.data.get('sequence')

        if lat is None or lon is None or sequence is None:
            return Response({'status': 'error', 'message': 'lat, lon, and sequence are required'},
                           status=status.HTTP_400_BAD_REQUEST)

        add_waypoint_to_recording(recording_id, lat, lon, sequence)
        return Response({'status': 'success'})
    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@api_view(['GET'])
def recordings_list(request):
    try:
        recordings = get_all_recordings()
        return Response({'status': 'success', 'recordings': recordings})
    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@api_view(['GET'])
def recording_waypoints(request, recording_id):
    try:
        waypoints = get_recording_waypoints(recording_id)
        return Response({'status': 'success', 'waypoints': waypoints})
    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@csrf_exempt
@api_view(['DELETE'])
def recording_delete(request, recording_id):
    try:
        delete_recording(recording_id)
        return Response({'status': 'success', 'deleted': recording_id})
    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@csrf_exempt
@api_view(['DELETE'])
def basic_waypoints_clear(request):
    try:
        clear_all_basic_waypoints()
        return Response({'status': 'success'})
    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@csrf_exempt
@api_view(['DELETE'])
def recordings_clear(request):
    try:
        clear_all_recordings()
        return Response({'status': 'success'})
    except Exception as e:
        return Response({'status': 'error', 'message': str(e)},
                       status=status.HTTP_500_INTERNAL_SERVER_ERROR)
