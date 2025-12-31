from enum import Enum
from math import pi
from typing import Union

# import rospy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.client import Client

from backend.input import filter_input, simulated_axis, safe_index, DeviceInputs
from backend.mappings import ControllerAxis, ControllerButton
from mrover.msg import Throttle

import logging
logger = logging.getLogger('django')

TAU = 2 * pi


class Joint(Enum):
    LINEAR_ACTUATOR = 0
    SENSOR_ACTUATOR = 1
    AUGER = 2
    PUMP_0 = 3
    PUMP_1 = 4


# The following are indexed with the values of the enum
JOINT_NAMES = [
    "linear_actuator",
    "sensor_actuator",
    "auger",
    "pump_0",
    "pump_1",
]

JOINT_SCALES = [
    -1.0,
    -1.0,
    1.0,
    1.0,
    1.0,
]

CONTROLLER_STICK_DEADZONE = 0.18


def subset(names: list[str], values: list[float], joints: set[Joint]) -> tuple[list[str], list[float]]:
    return [names[i.value] for i in joints], [values[i.value] for i in joints]

def compute_manual_joint_controls(controller: DeviceInputs) -> list[float]:
    return [
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.RIGHT_BUMPER, ControllerButton.LEFT_BUMPER),
            scale=JOINT_SCALES[Joint.LINEAR_ACTUATOR.value],
        ),
        0.0, #placeholder
        filter_input(
            simulated_axis(controller.axes, ControllerAxis.RIGHT_TRIGGER, ControllerAxis.LEFT_TRIGGER),
            scale=JOINT_SCALES[Joint.AUGER.value],
        ),
        0.0, #placeholder
        0.0, #placeholder
    ]


def send_sp_controls(inputs: DeviceInputs, sp_thr_pub: Publisher) -> None:
    throttle_msg = Throttle()
    manual_controls = compute_manual_joint_controls(inputs)
    joint_names, throttle_values = subset(JOINT_NAMES, manual_controls, set(Joint))
    throttle_msg.names = joint_names
    throttle_msg.throttles = throttle_values
    sp_thr_pub.publish(throttle_msg)
