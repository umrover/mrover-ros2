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
from mrover.srv import EnableBool

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

def compute_manual_joint_controls(controller: DeviceInputs, pump: int) -> list[float]:
    pump_controls = compute_pump_controls(controller, pump)

    controls = ([
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
        ),
    ] + pump_controls)
    
    print("Controls: ", controls)
    return controls


def send_sa_controls(sa_mode: str, pump: int, inputs: DeviceInputs, sa_thr_pub: Publisher) -> None:
    if(sa_mode == "disabled"):
        return
    throttle_msg = Throttle()
    manual_controls = compute_manual_joint_controls(inputs, pump)
    joint_names, throttle_values = subset(JOINT_NAMES, manual_controls, set(Joint))
    throttle_msg.names = joint_names
    throttle_msg.throttles = throttle_values
    sa_thr_pub.publish(throttle_msg)
    
def compute_pump_controls(inputs: DeviceInputs, pump: int) -> list[float]:
    sim_axis = filter_input(
        simulated_axis(inputs.buttons, ControllerButton.RIGHT_BUMPER, ControllerButton.LEFT_BUMPER), 
        scale = 1
    )
    if((sim_axis != 1.0) & (sim_axis != -1.0)):
        return [0.0, 0.0]
    if(pump == 0):
        return [sim_axis, 0.0]
    else:
        return [0.0, sim_axis]
