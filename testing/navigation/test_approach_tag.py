from testing.test_infra import MRoverTesting
from pathlib import Path

from launch import LaunchDescription

def action1(node):
    node.get_logger().info("hi")
    return True

def action2(node):
    node.get_logger().info("no")
    return False

def generate_launch_description():
    MRoverTesting.init()

    MRoverTesting.add_event(action1, 0, Path(__file__))
    MRoverTesting.add_event(action2, 0, Path(__file__))

    # MRoverTesting.add_node("nav.py", "navigation", ["navigation.yaml"])
    #
    # MRoverTesting.add_node("cost_map", "cost_map", ["perception.yaml"])

    return LaunchDescription(MRoverTesting.get_launch_actions())
