"""
API URL Configuration

This module defines all REST API endpoints for the teleoperation system.
"""
from django.urls import path
from .views import waypoints, auton, science, mast

app_name = 'api'

urlpatterns = [
    # Waypoints endpoints
    path('waypoints/basic/', waypoints.basic_waypoints_list, name='basic_waypoints_list'),
    path('waypoints/basic/save/', waypoints.basic_waypoints_save, name='basic_waypoints_save'),
    path('waypoints/basic/clear/', waypoints.basic_waypoints_clear, name='basic_waypoints_clear'),
    path('waypoints/auton/', waypoints.auton_waypoints_list, name='auton_waypoints_list'),
    path('waypoints/auton/save/', waypoints.auton_waypoints_save, name='auton_waypoints_save'),
    path('waypoints/basic/current/', waypoints.basic_course_current, name='basic_course_current'),
    path('waypoints/basic/current/save/', waypoints.basic_course_save, name='basic_course_save'),
    path('waypoints/auton/current/', waypoints.auton_course_current, name='auton_course_current'),
    path('waypoints/auton/current/save/', waypoints.auton_course_save, name='auton_course_save'),
    path('waypoints/auton/<str:waypoint_id>/', waypoints.auton_waypoint_delete, name='auton_waypoint_delete'),

    path('recordings/', waypoints.recordings_list, name='recordings_list'),
    path('recordings/create/', waypoints.recording_create, name='recording_create'),
    path('recordings/clear/', waypoints.recordings_clear, name='recordings_clear'),
    path('recordings/<int:recording_id>/', waypoints.recording_delete, name='recording_delete'),
    path('recordings/<int:recording_id>/waypoints/', waypoints.recording_waypoints, name='recording_waypoints'),
    path('recordings/<int:recording_id>/add/', waypoints.recording_add_waypoint, name='recording_add_waypoint'),

    # Auton endpoints
    path('auton/enable/', auton.enable_auton, name='enable_auton'),
    path('teleop/enable/', auton.enable_teleop, name='enable_teleop'),

    # Science endpoints
    path('science/gear-diff/position/', science.gear_diff_position, name='gear_diff_position'),

    # Mast endpoints
    path('mast/panorama/start/', mast.panorama_start, name='panorama_start'),
    path('mast/panorama/stop/', mast.panorama_stop, name='panorama_stop'),
    path('mast/gimbal/adjust/', mast.gimbal_adjust, name='gimbal_adjust'),
]
