from enum import Enum
from math import pi
from typing import Union

from rclpy.node import Node
from rclpy.publisher import Publisher

from lie import SE3
from backend.input import filter_input, simulated_axis, safe_index, DeviceInputs
from backend.mappings import ControllerAxis, ControllerButton
from mrover.msg import Throttle, IK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, Pose, Point, Quaternion

from tf2_ros.buffer import Buffer

import logging
logger = logging.getLogger('django')


TAU = 2 * pi

class Joint(Enum):
    A = 0
    B = 1
    C = 2
    DE_PITCH = 3
    DE_ROLL = 4
    GRIPPER = 5
    CAM = 6


# The following are indexed with the values of the enum

JOINT_NAMES = [
    "joint_a",
    "joint_b",
    "joint_c",
    "joint_de_pitch",
    "joint_de_roll",
    "gripper",
    "cam",
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
            simulated_axis(controller.buttons, ControllerButton.B, ControllerButton.X),
            scale=JOINT_SCALES[Joint.GRIPPER.value],
        ),
        filter_input(
            simulated_axis(controller.buttons, ControllerButton.Y, ControllerButton.A),
            scale=JOINT_SCALES[Joint.CAM.value],
        ),
    ]


def subset(names: list[str], values: list[float], joints: set[Joint]) -> tuple[list[str], list[float]]:
    return [names[i.value] for i in joints], [values[i.value] for i in joints]


def send_ra_controls(ra_mode: str, inputs: DeviceInputs, node: Node, thr_pub: Publisher, ee_pos_pub: Publisher, ee_vel_pub: Publisher, buffer: Buffer) -> None: 
    match ra_mode:
        case "throttle" | "ik-pos" | "ik-vel": #added filter for IK modes, hybrid removed
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
                    case "throttle":
                        throttle_msg = Throttle()
                        joint_names, throttle_values = subset(JOINT_NAMES, manual_controls, set(Joint))
                        throttle_msg.names = joint_names
                        throttle_msg.throttles = throttle_values
                        thr_pub.publish(throttle_msg)
                        
                    case "ik-pos":
                        ik_pos_msg = IK()
                        ik_pos_msg.target.header.stamp = node.get_clock().now().to_msg()
                        ik_pos_msg.target.header.frame_id = "base_link"
                        # gets se3 from TF tree
                        se3 = SE3.from_tf_tree(buffer, "base_link", "map")
                        tx, ty, tz = se3.translation()
                        qx, qy, qz, qw = se3.quat()
                        tx += (-1.0) * safe_index(inputs.axes, ControllerAxis.LEFT_Y)
                        ty += (-1.0) * safe_index(inputs.axes, ControllerAxis.LEFT_X)
                        tz += (-1.0) * safe_index(inputs.axes, ControllerAxis.RIGHT_Y)

                        # constructs pose
                        ik_pos_msg.target.pose = Pose(position=Point(x=tx, y=ty, z=tz), orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))
                        ee_pos_pub.publish(ik_pos_msg)
                    case "ik-vel":
                        ik_vel_msg = Vector3() 
                        #range -1 to 1 for each axis
                        ik_vel_msg.x = (-1.0) * safe_index(inputs.axes, ControllerAxis.LEFT_Y)
                        ik_vel_msg.y = (-1.0) * safe_index(inputs.axes, ControllerAxis.LEFT_X)
                        ik_vel_msg.z = (-1.0) * safe_index(inputs.axes, ControllerAxis.RIGHT_Y)
                        ee_vel_pub.publish(ik_vel_msg)

            else:
                if joint_positions:
                    de_pitch = joint_positions.position[joint_positions.name.index("joint_de_pitch")]
                    # position_publisher.publish(Position(["joint_de_roll", "joint_de_pitch"], [de_roll, de_pitch]))
