from enum import Enum
from math import pi
from typing import Union

from rclpy.node import Node
from rclpy.publisher import Publisher

from lie import SE3
from backend.input import filter_input, simulated_axis, safe_index, DeviceInputs
from backend.mappings import ControllerAxis, ControllerButton
from backend.ros_manager import get_node
from mrover.msg import Throttle, IK, Position, Velocity
from mrover.srv import IkMode
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Pose, Point, Quaternion

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


TAU = 2 * pi

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

JOINT_DE_POSITION_SCALE = 1

JOINT_A_MICRO_SCALE = 0.7

CONTROLLER_STICK_DEADZONE = 0.18

# Positions reported by the arm sensors
joint_positions: Union[JointState, None] = None


def joint_positions_callback(msg: JointState) -> None:
    global joint_positions
    joint_positions = msg


# rospy.Subscriber("arm_joint_data", JointState, joint_positions_callback)


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
    # filtered_joints = [j for j in joints if j.value < 5] # temporarily switch to this for sim testing
    filtered_joints = [j for j in joints]
    for i in filtered_joints:
        print(i)
    return [names[i.value] for i in filtered_joints], [values[i.value] for i in filtered_joints]


def send_ra_controls(
    inputs: DeviceInputs, node: Node, thr_pub: Publisher, ee_pos_pub: Publisher, ee_vel_pub: Publisher, buffer: Buffer
) -> None:
    global ra_mode
    match ra_mode:
        case "throttle" | "ik-pos" | "ik-vel":
            back_pressed = safe_index(inputs.buttons, ControllerButton.BACK) > 0.5
            forward_pressed = safe_index(inputs.buttons, ControllerButton.FORWARD) > 0.5
            home_pressed = safe_index(inputs.buttons, ControllerButton.HOME) > 0.5
            match back_pressed, forward_pressed, home_pressed:
                case True, False, False:
                    de_roll = -TAU / 4
                case False, True, False:
                    de_roll = TAU / 4
                case False, False, True:
                    de_roll = 0
                case _:
                    de_roll = None

            if de_roll is None:

                match ra_mode:
                    case "throttle":
                        manual_controls = compute_manual_joint_controls(inputs)
                        throttle_msg = Throttle()
                        joint_names, throttle_values = subset(JOINT_NAMES, manual_controls, set(Joint))
                        throttle_msg.names = joint_names
                        throttle_msg.throttles = throttle_values
                        arm_thr_pub.publish(throttle_msg)

                    case "ik-pos":
                        pos_msg = Position()
                        manual_controls = compute_manual_joint_controls(inputs)
                        joint_names, position_values = subset(JOINT_NAMES, manual_controls, set(Joint))
                        pos_msg.names = joint_names
                        pos_msg.positions = position_values
                        arm_pos_pub.publish(pos_msg)
                    case "ik-vel":
                        vel_msg = Velocity()
                        manual_controls = compute_manual_joint_controls(inputs)
                        joint_names, velocity_values = subset(JOINT_NAMES, manual_controls, set(Joint))
                        vel_msg.names = joint_names
                        vel_msg.velocities = velocity_values
                        arm_vel_pub.publish(vel_msg)

            else:
                if joint_positions:
                    de_pitch = joint_positions.position[joint_positions.name.index("joint_de_pitch")]
