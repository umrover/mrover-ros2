from pathlib import Path
import pickle
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from typing import Callable, Any, Union, TypeAlias
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
    OpaqueFunction
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals
import os

from enum import Enum

TEST_NODE_EXECUTABLE="test_node.py"
TEST_NODE_NAME="test_node"

class MRoverEventReturn(Enum):
    SUCCESS = 0
    PENDING = 1
    FAILURE = 2

class MRoverTesting:
    LaunchAction: TypeAlias = list[Union[Node, RegisterEventHandler, IncludeLaunchDescription, DeclareLaunchArgument]]
    _launch_actions: LaunchAction = [] # TODO: Keep up to date

    _node_map: dict[str, Node] = {}

    _events: list[TestEvent] = []

    @staticmethod
    def add_node(executable: str, name: str, parameters: list[Union[str, dict[str, LaunchConfiguration]]], use_debug: bool=False) -> None:
        # always give the node reference coordinates by default
        parameters.append("reference_coords.yaml")
        
        if executable == TEST_NODE_EXECUTABLE:
            temp_file_value = LaunchConfiguration('temp_file')
            parameters.append({'temp_file': temp_file_value})

        new_node = Node(
            package="mrover",
            executable=executable,
            name=name,
            parameters=[Path(get_package_share_directory("mrover"), "config", param_file) if type(param_file) is str else param_file for param_file in parameters],
            arguments=['--ros-args', '--log-level', 'DEBUG'] if use_debug else [],
        )

        MRoverTesting._launch_actions.append(new_node)
        
        MRoverTesting._node_map[name] = new_node

    @staticmethod
    def _send_events(_):
        msg_data = "\"{events: ["
        for event in MRoverTesting._events:
            data_data = "["
            for b in event.data:
                data_data += f"{b}, "
            data_data += "]"
            msg_data += f'{{function_name: {event.function_name}, module_spec: {event.module_spec}, event_group: {event.event_group}, data: {data_data}, timeout: {event.timeout} }},'
        msg_data += "]}\""
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
    def init(use_debug: bool=False):
        temp_file_arg = DeclareLaunchArgument(
            'temp_file', 
            default_value='bruh',
            description='A temporary file to write a return value'
        )

        MRoverTesting._launch_actions.append(temp_file_arg)

        MRoverTesting.add_node(TEST_NODE_EXECUTABLE, TEST_NODE_NAME, [], use_debug=use_debug)

        # https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Using-Event-Handlers.html
        test_start_handler = RegisterEventHandler(
            OnProcessStart(
                target_action=MRoverTesting._node_map[TEST_NODE_NAME],
                on_start=[
                    OpaqueFunction(function=MRoverTesting._send_events),
                ]
            )
        )

        test_end_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=MRoverTesting._node_map[TEST_NODE_NAME],
                on_exit=[
                    EmitEvent(event=Shutdown()),
                ]
            )
        )

        MRoverTesting._launch_actions.append(test_start_handler)
        MRoverTesting._launch_actions.append(test_end_handler)

        pass

    @staticmethod
    def enable_sim(headless: bool=True) -> None:
        simulator_node = Node(
            package="mrover",
            executable="simulator",
            name="simulator",
            parameters=[
                Path(get_package_share_directory("mrover"), "config", "simulator.yaml"),
                Path(get_package_share_directory("mrover"), "config", "reference_coords.yaml"),
                {"headless": headless},
            ],
        )

        launch_include_base = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("mrover"), "launch/base.launch.py"))
        )

        MRoverTesting._launch_actions.append(simulator_node)
        MRoverTesting._launch_actions.append(launch_include_base)

    @staticmethod
    def get_launch_actions() -> LaunchAction: # TODO(john): Keep this line up to date with all of the things that can be included in the launch descriptor
        return MRoverTesting._launch_actions

    @staticmethod
    def add_event(func: Callable[[rclpy.node.Node, ...], MRoverEventReturn], timeout: int, **kwargs) -> None:
        event = TestEvent(
                    module_spec = str(Path(func.__code__.co_filename).relative_to(Path(get_package_share_directory("mrover"))).with_suffix('')).replace('/', '.'),
                    function_name = func.__name__,
                    timeout = timeout,
                    event_group = len(MRoverTesting._events),
                    data = pickle.dumps(kwargs)
                )

        MRoverTesting._events.append(event)

    @staticmethod
    def add_parallel_events(funcs: list[Callable[[rclpy.node.Node, ...], MRoverEventReturn]], timeout: int, args: list[dict[str, Any]]) -> None:
        # I pass a list of dictionaries here, it doesn't yield as nice of syntax, but I am not sure a better alternative
        assert(len(funcs) == len(args))
        group_event_idx = len(MRoverTesting._events)
        for func, arg in zip(funcs, args):

            event = TestEvent(
                        module_spec = str(Path(func.__code__.co_filename).relative_to(Path(get_package_share_directory("mrover"))).with_suffix('')).replace('/', '.'),
                        function_name = func.__name__,
                        timeout = timeout, # this is only required for the first event, all others will be discarded by the ROS test node
                        event_group = group_event_idx,
                        data = pickle.dumps(arg)
                    )

            MRoverTesting._events.append(event)
