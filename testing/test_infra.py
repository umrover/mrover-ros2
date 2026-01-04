from pathlib import Path
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from typing import Callable
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.event_handlers import (
    OnExecutionComplete,
    OnProcessExit,
    OnProcessIO,
    OnProcessStart,
    OnShutdown
)
from launch.events import Shutdown
from launch.substitutions import (
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    LocalSubstitution,
    PythonExpression
)
from launch_ros.actions import Node

import rclpy
import rclpy.node

import importlib.util

from mrover.msg import TestEvent

class MRoverTesting:
    _launch_actions: list[Node] = [] # TODO: Keep up to date

    _node_map: dict[str, Node] = {}

    _events: list[TestEvent] = []

    @staticmethod
    def add_node(executable: str, name: str, parameters: list[str]) -> None:
        # always give the node reference coordinates by default
        parameters.append("reference_coords.yaml")

        new_node = Node(
            package="mrover",
            executable=executable,
            name=name,
            parameters=[Path(get_package_share_directory("mrover"), "config", param_file) for param_file in parameters]
        )

        MRoverTesting._launch_actions.append(new_node)
        
        MRoverTesting._node_map[name] = new_node

    @staticmethod
    def _send_events(context):
        msg_data = "\"{events: ["
        for event in MRoverTesting._events:
            msg_data += f'{{function_name: {event.function_name}, module_spec: {event.module_spec} }},'
        msg_data += "]}\""
        rclpy.logging.get_logger("bruh").info(msg_data)
        event_msg = ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                ' topic pub --once ',
                '/test_events ',
                'mrover/msg/TestEvents ',
                msg_data
            ]],
            shell=True
        )

        return [event_msg]

    @staticmethod
    def init():
        MRoverTesting.add_node("test_node.py", "test_node", [])

        # https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Using-Event-Handlers.html
        test_start_handler = RegisterEventHandler(
            OnProcessStart(
                target_action=MRoverTesting._node_map["test_node"],
                on_start=[
                    OpaqueFunction(function=MRoverTesting._send_events),
                ]
            )
        )

        MRoverTesting._launch_actions.append(test_start_handler)

        pass


    @staticmethod
    def get_launch_actions() -> list[Node]: # TODO(john): Keep this line up to date with all of the things that can be included in the launch descriptor
        return MRoverTesting._launch_actions

    @staticmethod
    def add_event(func: Callable[[rclpy.node.Node], bool], timeout: int, file: Path) -> None:
        event = TestEvent(
                    module_spec = str(file.relative_to(Path(get_package_share_directory("mrover"))).with_suffix('')).replace('/', '.'),
                    function_name = func.__name__
                )

        MRoverTesting._events.append(event)
