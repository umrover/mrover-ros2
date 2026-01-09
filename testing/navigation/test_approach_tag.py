from mrover.msg import GPSWaypoint, WaypointType, StateMachineStateUpdate

from testing.test_infra import MRoverTesting, MRoverEventReturn

from pathlib import Path

from launch import LaunchDescription

from mrover.srv import EnableAuton

import rclpy

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

    return LaunchDescription(MRoverTesting.get_launch_actions())
