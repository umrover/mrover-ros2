from enum import Enum

from rclpy.publisher import Publisher

from backend.input import filter_input, simulated_axis, safe_index, DeviceInputs
from backend.mappings import ControllerButton
from mrover.msg import Throttle


class Joint(Enum):
    LINEAR_ACTUATOR = 0
    AUGER = 1
    BRUSH = 2


JOINT_NAMES = [
    "linear_actuator",
    "auger",
    "brush",
]

JOINT_SCALES = [
    -1.0,
    1.0,
    1.0,
]

brush_active: bool = False
prev_brush_button: bool = False


def compute_manual_joint_controls(controller: DeviceInputs) -> list[float]:
    global brush_active, prev_brush_button

    brush_pressed = bool(safe_index(controller.buttons, ControllerButton.LEFT_BUMPER))
    if brush_pressed and not prev_brush_button:
        brush_active = not brush_active
    prev_brush_button = brush_pressed

    return [
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.DPAD_DOWN, ControllerButton.DPAD_UP),
            scale=JOINT_SCALES[Joint.LINEAR_ACTUATOR.value],
        ),
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.LEFT_TRIGGER, ControllerButton.RIGHT_TRIGGER),
            scale=JOINT_SCALES[Joint.AUGER.value],
        ),
        JOINT_SCALES[Joint.BRUSH.value] if brush_active else 0.0,
    ]


def send_sp_controls(inputs: DeviceInputs, sp_thr_pub: Publisher) -> None:
    throttle_msg = Throttle()
    manual_controls = compute_manual_joint_controls(inputs)
    throttle_msg.names = list(JOINT_NAMES)
    throttle_msg.throttles = manual_controls
    sp_thr_pub.publish(throttle_msg)
