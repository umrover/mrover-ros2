from enum import Enum
from math import pi
from typing import Union

# import rospy
from rclpy.node import Node
from rclpy.publisher import Publisher

from backend.input import filter_input, simulated_axis, safe_index, DeviceInputs
from backend.mappings import ControllerAxis, ControllerButton
from mrover.msg import Throttle

import logging
logger = logging.getLogger('django')

TAU = 2 * pi


class Joint(Enum):
    # SA_X = 0
    # SA_Y = 1
    # SA_Z = 2
    # SAMPLER = 3
    # SENSOR_ACTUATOR = 4
    LINEAR_ACTUATOR = 0
    SENSOR_ACTUATOR = 1
    AUGER = 2


# The following are indexed with the values of the enum

JOINT_NAMES = [
    # "sa_x",
    # "sa_y",
    # "sa_z",
    # "sampler",
    # "sensor_actuator",
    "linear_actuator", 
    "sensor_actuator", 
    "auger"
]

JOINT_SCALES = [
    # -1.0,
    # -1.0,
    # 1.0,
    # 1.0,
    # 1.0,
    -1.0, 
    -1.0, 
    1.0, 
]

CONTROLLER_STICK_DEADZONE = 0.18


def subset(names: list[str], values: list[float], joints: set[Joint]) -> tuple[list[str], list[float]]:
    return [names[i.value] for i in joints], [values[i.value] for i in joints]

def compute_manual_joint_controls(controller: DeviceInputs) -> list[float]:
    return [
        # filter_input(
        #     safe_index(controller.axes, ControllerAxis.LEFT_Y),
        #     quadratic=True,
        #     scale=JOINT_SCALES[Joint.SA_X.value],
        #     deadzone=CONTROLLER_STICK_DEADZONE,
        # ),
        # filter_input(
        #     safe_index(controller.axes, ControllerAxis.LEFT_X),
        #     quadratic=True,
        #     scale=JOINT_SCALES[Joint.SA_Y.value],
        #     deadzone=CONTROLLER_STICK_DEADZONE,
        # ),
        # filter_input(
        #     safe_index(controller.axes, ControllerAxis.RIGHT_Y),
        #     quadratic=True,
        #     scale=JOINT_SCALES[Joint.SA_Z.value],
        #     deadzone=CONTROLLER_STICK_DEADZONE,
        # ),
        # filter_input(
        #     simulated_axis(controller.buttons, ControllerButton.RIGHT_BUMPER, ControllerButton.LEFT_BUMPER),
        #     scale=JOINT_SCALES[Joint.SAMPLER.value],
        # ),
        # filter_input(
        #     simulated_axis(controller.buttons, ControllerButton.RIGHT_TRIGGER, ControllerButton.LEFT_TRIGGER),
        #     scale=JOINT_SCALES[Joint.SENSOR_ACTUATOR.value],
        # ),
        filter_input(
            safe_index(controller.axes, ControllerAxis.LEFT_Y), 
            quadratic=True, 
            scale=JOINT_SCALES[Joint.LINEAR_ACTUATOR.value], 
            deadzone=CONTROLLER_STICK_DEADZONE, 
        ), 
        filter_input(
            safe_index(controller.axes, ControllerAxis.RIGHT_Y), 
            quadratic=True, 
            scale=JOINT_SCALES[Joint.SENSOR_ACTUATOR.value], 
            deadzone=CONTROLLER_STICK_DEADZONE, 
        ), 
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.RIGHT_TRIGGER, ControllerButton.LEFT_TRIGGER),
            scale=JOINT_SCALES[Joint.AUGER.value],
        )
    ]


def send_sa_controls(inputs: DeviceInputs, sa_thr_pub: Publisher) -> None:
    # manual_controls = compute_manual_joint_controls(inputs)
    throttle_msg = Throttle()
    manual_controls = compute_manual_joint_controls(inputs)
    joint_names, throttle_values = subset(JOINT_NAMES, manual_controls, set(Joint))
    throttle_msg.names = joint_names
    throttle_msg.throttles = throttle_values
    logger.debug("publishing")
    sa_thr_pub.publish(throttle_msg)
    
