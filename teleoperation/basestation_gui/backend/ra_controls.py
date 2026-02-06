import asyncio
from enum import Enum
import threading

from rclpy.node import Node
from rclpy.publisher import Publisher

from backend.input import filter_input, simulated_axis, safe_index, DeviceInputs
from backend.mappings import ControllerAxis, ControllerButton
from backend.managers.ros import get_service_client
from mrover.msg import Throttle, IK
from mrover.srv import IkMode
from geometry_msgs.msg import Twist

from tf2_ros.buffer import Buffer

ra_mode = "disabled"
ra_mode_lock = threading.Lock()


def get_ra_mode() -> str:
    with ra_mode_lock:
        return ra_mode


async def set_ra_mode(new_ra_mode: str):
    global ra_mode
    with ra_mode_lock:
        ra_mode = new_ra_mode

    if new_ra_mode == "ik-pos":
        await call_ik_mode_service(IK_MODE_POSITION_CONTROL)
    elif new_ra_mode == "ik-vel":
        await call_ik_mode_service(IK_MODE_VELOCITY_CONTROL)


async def call_ik_mode_service(mode: int) -> bool:
    client = get_service_client(IkMode, "/ik_mode")

    try:
        service_ready = await asyncio.wait_for(
            asyncio.get_event_loop().run_in_executor(
                None, lambda: client.wait_for_service(timeout_sec=0.1)
            ),
            timeout=1.0
        )
        if not service_ready:
            return False
    except asyncio.TimeoutError:
        return False

    request = IkMode.Request()
    request.mode = mode

    future = client.call_async(request)
    try:
        await asyncio.wait_for(
            asyncio.get_event_loop().run_in_executor(None, future.result),
            timeout=5.0
        )
        result = future.result()
        return result.success if result else False
    except asyncio.TimeoutError:
        return False
    except Exception:
        return False


IK_MODE_POSITION_CONTROL = 0
IK_MODE_VELOCITY_CONTROL = 1
IK_MODE_TYPING = 2


class Joint(Enum):
    A = 0
    B = 1
    C = 2
    DE_PITCH = 3
    DE_ROLL = 4
    CAM = 5
    GRIPPER = 6


# The following are indexed with the values of the enum

JOINT_NAMES = [
    "joint_a",
    "joint_b",
    "joint_c",
    "joint_de_pitch",
    "joint_de_roll",
    "cam",
    "gripper",
]

JOINT_SCALES = [
    -1.0,
    0.8,
    1.0,
    -1.0,
    1.0,
    1.0,
    1.0,
]

JOINT_A_MICRO_SCALE = 0.7

CONTROLLER_STICK_DEADZONE = 0.18


def compute_manual_joint_controls(controller: DeviceInputs) -> list[float]:
    return [
        filter_input(
            safe_index(controller.axes, ControllerAxis.LEFT_X),
            quadratic=True,
            scale=JOINT_SCALES[Joint.A.value],
            deadzone=CONTROLLER_STICK_DEADZONE,
        )
        + filter_input(
            simulated_axis(controller.buttons, ControllerButton.DPAD_LEFT, ControllerButton.DPAD_RIGHT),
            scale=JOINT_A_MICRO_SCALE,
        ),
        filter_input(
            safe_index(controller.axes, ControllerAxis.LEFT_Y),
            quadratic=True,
            scale=JOINT_SCALES[Joint.B.value],
            deadzone=CONTROLLER_STICK_DEADZONE,
        ),
        filter_input(
            safe_index(controller.axes, ControllerAxis.RIGHT_Y),
            quadratic=True,
            scale=JOINT_SCALES[Joint.C.value],
            deadzone=CONTROLLER_STICK_DEADZONE,
        ),
        filter_input(
            simulated_axis(controller.axes, ControllerAxis.RIGHT_TRIGGER, ControllerAxis.LEFT_TRIGGER),
            scale=JOINT_SCALES[Joint.DE_PITCH.value],
        ),
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.RIGHT_BUMPER, ControllerButton.LEFT_BUMPER),
            scale=JOINT_SCALES[Joint.DE_ROLL.value],
        ),
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.Y, ControllerButton.A),
            scale=JOINT_SCALES[Joint.CAM.value],
        ),
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.B, ControllerButton.X),
            scale=JOINT_SCALES[Joint.GRIPPER.value],
        ),
    ]


def subset(names: list[str], values: list[float], joints: set[Joint]) -> tuple[list[str], list[float]]:
    return [names[i.value] for i in joints], [values[i.value] for i in joints]


def send_ra_controls(
    inputs: DeviceInputs, node: Node, thr_pub: Publisher, ee_pos_pub: Publisher, ee_vel_pub: Publisher, buffer: Buffer
) -> None:
    current_mode = get_ra_mode()
    match current_mode:
        case "throttle" | "ik-pos" | "ik-vel":
            match current_mode:
                case "throttle":
                    manual_controls = compute_manual_joint_controls(inputs)
                    throttle_msg = Throttle()
                    joint_names, throttle_values = subset(JOINT_NAMES, manual_controls, set(Joint))
                    throttle_msg.names = joint_names
                    throttle_msg.throttles = throttle_values
                    thr_pub.publish(throttle_msg)

                case "ik-pos":
                    ik_pos_msg = IK()
                    ik_pos_msg.pos.x = (-1.0) * safe_index(inputs.axes, ControllerAxis.LEFT_Y)
                    ik_pos_msg.pos.y = (-1.0) * safe_index(inputs.axes, ControllerAxis.LEFT_X)
                    ik_pos_msg.pos.z = (-1.0) * safe_index(inputs.axes, ControllerAxis.RIGHT_Y)
                    ik_pos_msg.pitch = 1.0 * simulated_axis(inputs.axes, ControllerAxis.RIGHT_TRIGGER, ControllerAxis.LEFT_TRIGGER)
                    ik_pos_msg.roll = 1.0 * simulated_axis(inputs.buttons, ControllerButton.RIGHT_BUMPER, ControllerButton.LEFT_BUMPER)
                    ee_pos_pub.publish(ik_pos_msg)
                case "ik-vel":
                    ik_vel_msg = Twist()
                    ik_vel_msg.linear.x = (-1.0) * filter_input(safe_index(inputs.axes, ControllerAxis.LEFT_Y), deadzone=CONTROLLER_STICK_DEADZONE)
                    ik_vel_msg.linear.y = (-1.0) * filter_input(safe_index(inputs.axes, ControllerAxis.LEFT_X), deadzone=CONTROLLER_STICK_DEADZONE)
                    ik_vel_msg.linear.z = (-1.0) * filter_input(safe_index(inputs.axes, ControllerAxis.RIGHT_Y), deadzone=CONTROLLER_STICK_DEADZONE)
                    ik_vel_msg.angular.y = 1.0 * simulated_axis(inputs.buttons, ControllerButton.RIGHT_BUMPER, ControllerButton.LEFT_BUMPER)
                    ik_vel_msg.angular.x = 1.0 * simulated_axis(inputs.axes, ControllerAxis.RIGHT_TRIGGER, ControllerAxis.LEFT_TRIGGER)
                    ee_vel_pub.publish(ik_vel_msg)

                    cam_throttle = filter_input(
                        simulated_axis(inputs.buttons, ControllerButton.Y, ControllerButton.A),
                        scale=JOINT_SCALES[Joint.CAM.value],
                    )
                    throttle_msg = Throttle()
                    throttle_msg.names = ["cam"]
                    throttle_msg.throttles = [cam_throttle]
                    thr_pub.publish(throttle_msg)
