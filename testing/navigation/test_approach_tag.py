from mrover.msg import GPSWaypoint, WaypointType, StateMachineStateUpdate
from tf2_ros import BufferClient

from testing.test_infra import MRoverTesting, MRoverEventReturn

from pathlib import Path

from launch import LaunchDescription

from mrover.srv import EnableAuton

import rclpy

from manifpy import SE3

from lie.conversions import from_position_orientation, from_tf_tree

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np

ROVER_TF_FRAME = "base_link"
MAP_TF_FRAME = "map"

def enable_auton(node, waypoints: list[GPSWaypoint]) -> MRoverEventReturn:
    enable_auton_service = node.create_client(EnableAuton, "enable_auton")

    while not enable_auton_service.wait_for_service(timeout_sec=100000.0):
        node.get_logger().info('/enable_auton service not available, waiting again...')
        return MRoverEventReturn.FAILURE

    req = EnableAuton.Request()
    req.enable = True
    req.waypoints = waypoints
    
    _ = enable_auton_service.call_async(req)

    return MRoverEventReturn.SUCCESS

navigation_state_assertion = ""
has_navigation_reached_asserted_state = False

def assert_navigation_state_callback(msg: StateMachineStateUpdate):
    global has_navigation_reached_asserted_state, navigation_state_assertion

    # if we reach the desired state, then set flag to true
    if navigation_state_assertion == msg.state:
        has_navigation_reached_asserted_state = True

def assert_navigation_state(node, state: str):
    # no need to check if this is already defined since it is a write only operation
    # should be set before the callback is defined
    global navigation_state_assertion, has_navigation_reached_asserted_state

    if navigation_state_assertion == "":
        navigation_state_assertion = state
        has_navigation_reached_asserted_state = False

    # create the subscriber if it hasn't already
    if not hasattr(node, 'navigation_state_subscriber'):
        node.navigation_state_subscriber = node.create_subscription(StateMachineStateUpdate, 'nav_state', assert_navigation_state_callback, 10)

    if has_navigation_reached_asserted_state:
        navigation_state_assertion = ""
        return MRoverEventReturn.SUCCESS

    return MRoverEventReturn.PENDING

def assert_rover_location(node, tx: float=0, ty: float=0, tz:float =0, qx: float=0, qy: float=0, qz: float=0, qw: float=1, translation_error: float = 0.1, rotational_error: float = 0.1):
    if not hasattr(node, 'tf_buffer'):
        node.tf_buffer = Buffer()
        node.tf_listener = TransformListener(node.tf_buffer, node)

    target: SE3 = from_position_orientation(tx, ty, tz, qx, qy, qz, qw)

    rover_pos: SE3 = from_tf_tree(node.tf_buffer, ROVER_TF_FRAME, MAP_TF_FRAME)

    # assert translations
    is_rover_in_location_translation = [np.abs(target_component - current_component) < translation_error for target_component, current_component in zip(target.translation(), rover_pos.translation())]
    is_rover_in_location_rotation = [np.abs(target_component - current_component) < rotational_error for target_component, current_component in zip(target.quat(), rover_pos.quat())]

    if all(is_rover_in_location_rotation) and all(is_rover_in_location_translation):
        return MRoverEventReturn.SUCCESS

    return MRoverEventReturn.PENDING


def generate_launch_description():
    MRoverTesting.init()

    MRoverTesting.enable_sim(headless=True)

    MRoverTesting.add_node("nav.py", "navigation", ["navigation.yaml"])

    MRoverTesting.add_node("cost_map", "cost_map", ["perception.yaml"])

    MRoverTesting.add_node("stereo_tag_detector", "stereo_tag_detector", [])

    MRoverTesting.add_event(enable_auton, 1, waypoints=[
        GPSWaypoint(
                tag_id = 0,
                enable_costmap = True,
                latitude_degrees = 42.29321060337751,
                longitude_degrees = -83.70957248114468,
                type = WaypointType(val=WaypointType.POST)
            )
        ])

    # I wanted this "DoneState" to be DoneState() from an import but it complicates the environments...
    # this implies that there is no way to import from the actual navigation library since it will fail to find the modules
    MRoverTesting.add_event(assert_navigation_state, 5, state="WaypointState")
    MRoverTesting.add_parallel_events([assert_navigation_state, assert_navigation_state], 30, [{"state": "ApproachTargetState"}, {"state": "DoneState"}])
    MRoverTesting.add_event(assert_rover_location, 5, 
        tx=9.738519668579102,
        ty=2.573899030685425,
        tz=-0.016374515369534492,
        qx=0.00019315559442369901,
        qy=-0.0009829838806873692,
        qz=-0.15341376204409993,
        qw=0.9881615323665879
    )

    return LaunchDescription(MRoverTesting.get_launch_actions())
