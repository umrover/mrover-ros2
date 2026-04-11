from enum import Enum

from rclpy.publisher import Publisher

from backend.input import filter_input, simulated_axis, DeviceInputs
from backend.mappings import ControllerButton
from mrover.msg import Throttle


class Joint(Enum):
    LINEAR_ACTUATOR = 0
    AUGER = 1


JOINT_NAMES = [
    "linear_actuator",
    "auger",
]

JOINT_SCALES = [
    -1.0,
    1.0,
]


def compute_manual_joint_controls(controller: DeviceInputs) -> list[float]:
    return [
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.DPAD_UP, ControllerButton.DPAD_DOWN),
            scale=JOINT_SCALES[Joint.LINEAR_ACTUATOR.value],
        ),
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.RIGHT_TRIGGER, ControllerButton.LEFT_TRIGGER),
            scale=JOINT_SCALES[Joint.AUGER.value],
        ),
    ]


def send_sp_controls(inputs: DeviceInputs, sp_thr_pub: Publisher) -> None:
    throttle_msg = Throttle()
    manual_controls = compute_manual_joint_controls(inputs)
    throttle_msg.names = list(JOINT_NAMES)
    throttle_msg.throttles = manual_controls
    sp_thr_pub.publish(throttle_msg)
