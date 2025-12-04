from enum import Enum

from rclpy.node import Node
from rclpy.publisher import Publisher

from backend.input import filter_input, simulated_axis, safe_index, DeviceInputs
from backend.mappings import ControllerAxis, ControllerButton
from backend.ros_manager import get_node
from mrover.msg import Throttle, IK
from mrover.srv import IkMode
from geometry_msgs.msg import Twist

from tf2_ros.buffer import Buffer

import time

ra_mode = "disabled"

def get_ra_mode() -> str:
    return ra_mode


def set_ra_mode(new_ra_mode: str):
    global ra_mode
    ra_mode = new_ra_mode

    node = get_node()

    if new_ra_mode == "ik-pos":
        call_ik_mode_service(node, IK_MODE_POSITION_CONTROL)
    elif new_ra_mode == "ik-vel":
        call_ik_mode_service(node, IK_MODE_VELOCITY_CONTROL)


def call_ik_mode_service(node: Node, mode: int) -> bool:
    client = node.create_client(IkMode, "/ik_mode")

    if not client.wait_for_service(timeout_sec=1.0):
        return False

    request = IkMode.Request()
    request.mode = mode

    future = client.call_async(request)
    start_time = time.time()
    while not future.done():
        if time.time() - start_time > 5.0:
            return False
        time.sleep(0.01)

    result = future.result()
    return result.success if result else False


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
            simulated_axis(controller.buttons, ControllerButton.RIGHT_TRIGGER, ControllerButton.LEFT_TRIGGER),
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
    filtered_joints = [j for j in joints]
    return [names[i.value] for i in filtered_joints], [values[i.value] for i in filtered_joints]


def send_ra_controls(
    inputs: DeviceInputs, node: Node, thr_pub: Publisher, ee_pos_pub: Publisher, ee_vel_pub: Publisher, buffer: Buffer
) -> None:
    match ra_mode:
        case "throttle" | "ik-pos" | "ik-vel":
            match ra_mode:
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
                    ik_pos_msg.pitch = 1.0 * simulated_axis(inputs.buttons, ControllerButton.RIGHT_TRIGGER, ControllerButton.LEFT_TRIGGER)
                    ik_pos_msg.roll = 1.0 * simulated_axis(inputs.buttons, ControllerButton.RIGHT_BUMPER, ControllerButton.LEFT_BUMPER)
                    ee_pos_pub.publish(ik_pos_msg)
                case "ik-vel":
                    ik_vel_msg = Twist()
                    ik_vel_msg.linear.x = (-1.0) * filter_input(safe_index(inputs.axes, ControllerAxis.LEFT_Y), deadzone=CONTROLLER_STICK_DEADZONE)
                    ik_vel_msg.linear.y = (-1.0) * filter_input(safe_index(inputs.axes, ControllerAxis.LEFT_X), deadzone=CONTROLLER_STICK_DEADZONE)
                    ik_vel_msg.linear.z = (-1.0) * filter_input(safe_index(inputs.axes, ControllerAxis.RIGHT_Y), deadzone=CONTROLLER_STICK_DEADZONE)
                    ik_vel_msg.angular.y = 1.0 * simulated_axis(inputs.buttons, ControllerButton.RIGHT_BUMPER, ControllerButton.LEFT_BUMPER)
                    ik_vel_msg.angular.x = 1.0 * simulated_axis(inputs.buttons, ControllerButton.RIGHT_TRIGGER, ControllerButton.LEFT_TRIGGER)
                    ee_vel_pub.publish(ik_vel_msg)

                    manual_controls = compute_manual_joint_controls(inputs)
                    throttle_msg = Throttle()
                    joint_names, throttle_values = subset(JOINT_NAMES, manual_controls, set(Joint))
                    throttle_msg.names = ["cam"]
                    throttle_msg.throttles = [throttle_values[joint_names.index("cam")]]
                    thr_pub.publish(throttle_msg)
