from enum import Enum
from math import pi
from typing import Union

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.publisher import Publisher

from lie import SE3
# import rospy
from backend.input import filter_input, simulated_axis, safe_index, DeviceInputs
from backend.mappings import ControllerAxis, ControllerButton
from mrover.msg import Throttle, Position
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

import tf2_ros
from tf2_ros.buffer import Buffer
buffer = Buffer()

TAU = 2 * pi

# rospy.init_node("teleoperation", disable_signals=True)

# throttle_publisher = rospy.Publisher("arm_throttle_cmd", Throttle, queue_size=1) FOR MANUAL
# position_publisher = rospy.Publisher("arm_position_cmd", Position, queue_size=1) for IK


class Joint(Enum):
    A = 0
    B = 1
    C = 2
    DE_PITCH = 3
    DE_ROLL = 4
    ALLEN_KEY = 5
    GRIPPER = 6


# The following are indexed with the values of the enum

JOINT_NAMES = [
    "joint_a",
    "joint_b",
    "joint_c",
    "joint_de_pitch",
    "joint_de_roll",
    "allen_key",
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
            scale=JOINT_SCALES[Joint.ALLEN_KEY.value],
        ),
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.B, ControllerButton.X),
            scale=JOINT_SCALES[Joint.GRIPPER.value],
        ),
    ]


def subset(names: list[str], values: list[float], joints: set[Joint]) -> tuple[list[str], list[float]]:
    return [names[i.value] for i in joints], [values[i.value] for i in joints]


def send_ra_controls(ra_mode: str, inputs: DeviceInputs, node: Node, thr_pub: Publisher, pos_pub: Publisher) -> None: 
    match ra_mode:
        case "manual" | "ik": #added filter for IK mode, hybrid removed
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
                manual_controls = compute_manual_joint_controls(inputs)

                match ra_mode:
                    case "manual":
                        throttle_msg = Throttle()
                        joint_names, throttle_values = subset(JOINT_NAMES, manual_controls, set(Joint))
                        throttle_msg.names = joint_names
                        throttle_msg.throttles = throttle_values
                        thr_pub.publish(throttle_msg)
                        
                    case "ik":
                        ik_msg = PoseStamped()
                        ik_msg.header.stamp = node.get_clock().now().to_msg()
                        ik_msg.header.frame_id = "base_link"
                        ik_msg.pose = SE3.from_tf_tree(buffer, "base_link", "map")
                        pos_pub.publish(ik_msg)

            else:
                if joint_positions:
                    de_pitch = joint_positions.position[joint_positions.name.index("joint_de_pitch")]
                    # position_publisher.publish(Position(["joint_de_roll", "joint_de_pitch"], [de_roll, de_pitch]))
