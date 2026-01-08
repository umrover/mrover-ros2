#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pathlib import Path

from rclpy.logging import get_logger

from std_msgs.msg import String
from test_infra import MRoverEventReturn, MRoverTesting
from rcl_interfaces.msg import ParameterDescriptor

from mrover.msg import TestEvent, TestEvents
from typing import Optional

import types
import importlib
import pickle
import signal

NODE_NAME='test_node'
TEMP_FILE_PATH: Path = Path("")

def exit_error(msg: str = "") -> None:
    if msg != "":
        get_logger(NODE_NAME).error(msg)

    with open(TEMP_FILE_PATH, "w") as f:
        f.write("1")

    exit(1)

def exit_success(msg: str = "") -> None:
    if msg != "":
        get_logger(NODE_NAME).info(msg)

    with open(TEMP_FILE_PATH, "w") as f:
        f.write("0")

    exit(0)

def timeout_handler(signum: int, frame: Optional[types.FrameType]):
    exit_error("Timeout occurred...")

class TestNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.index = 0
        self.events = []
        self.timeouts = []
        self.args = []
        self.event_subscriber = self.create_subscription(
            TestEvents,
            'test_events',
            self.event_callback,
            10)

        signal.signal(signal.SIGALRM, timeout_handler)

        self.event_subscriber

        self.is_first_event_call = True

        self.declare_parameter('temp_file', 'iwiwiwi', ParameterDescriptor(description='Temporary file...'))
        global TEMP_FILE_PATH
        TEMP_FILE_PATH = Path(str(self.get_parameter('temp_file').value))

    def timer_callback(self):
        self.get_logger().debug(f"Events length: {len(self.events)}")

        # the functionality implemented here is actually very similar to ROS2 callbacks
        # this could be switched to create new timer callbacks inside of the ros infra
        # I think that could be a little bit cooked though since it would just make a crazy number
        # of callbacks happen
        if len(self.events) == 0:
            self.get_logger().debug("No Events Loaded...")

        elif self.index < len(self.events):
            assert(len(self.args[self.index]) == len(self.events[self.index]))
            assert(len(self.args) == len(self.events) == len(self.timeouts))

            if self.is_first_event_call:
                self.is_first_event_call = False
                self.get_logger().info(f"{self.timeouts[self.index]}")
                signal.alarm(self.timeouts[self.index])
                self.is_event_done = [False] * len(self.args[self.index]) # TODO: Resume here with addding parallel events

            for idx, (args, event) in enumerate(zip(self.args[self.index], self.events[self.index])):
                self.get_logger().debug(f"Running event index: {self.index}.{idx} func: {event.__name__}")
                return_val: MRoverEventReturn = event(self, **pickle.loads(args))
                match return_val.value:
                    case MRoverEventReturn.SUCCESS.value:
                        self.is_event_done[idx] = True

                        if all(self.is_event_done):

                            # clear any active alarms
                            signal.alarm(0)
                            self.is_first_event_call = True

                            # increment on to the nexte event
                            self.index += 1
                            return
                    case MRoverEventReturn.FAILURE.value:
                        # clear any active alarms
                        signal.alarm(0)
                        self.is_first_event_call = True

                        # no need to check all of the other statuses since we need all of them to succeed for the parallel event to succeed
                        # TODO: add more descriptive failures
                        exit_error("Event Exited With Failure...")
                    case MRoverEventReturn.PENDING.value:
                        return
                    case _:
                        exit_error("Event Unknown Recieved...")
        elif self.index == len(self.events):
            exit_success("Events Finished...")

    def event_callback(self, msg: TestEvents) -> None:
        for event in msg.events:
            try:
                module = importlib.import_module(event.module_spec)

                function = getattr(module, event.function_name)

                # append case
                if event.event_group >= len(self.events):
                    self.events.append([function])
                    self.args.append([event.data])
                    self.timeouts.append(event.timeout)
                else:
                    self.events[event.event_group].append(function)
                    self.args[event.event_group].append(event.data)

                    # there should already be a timeout associated with the event group
                    assert(len(self.timeouts) >= event.event_group)

            except ImportError:
                get_logger('testing').error("Error importing the module...")
            except AttributeError:
                get_logger('testing').error("Error getting the function attribute...")

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TestNode()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
