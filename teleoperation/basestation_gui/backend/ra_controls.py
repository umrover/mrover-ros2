import asyncio
import math
from enum import Enum
import threading

import rclpy.time
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.publisher import Publisher

from backend.input import filter_input, simulated_axis, safe_index, DeviceInputs
from backend.mappings import ControllerAxis, ControllerButton
from backend.managers.ros import get_service_client
from backend.database import get_config, set_config
from backend.utils.ros_service import call_service_async
from lie import SE3
from mrover.msg import Throttle, IK
from mrover.srv import IkMode
from geometry_msgs.msg import Twist

STOW_CONFIG_KEY = "arm.stow_position"

ra_mode = "disabled"
ra_mode_lock = threading.Lock()

ik_pos_pub: Publisher | None = None
stow_task: asyncio.Task | None = None
tf_buffer: tf2_ros.Buffer | None = None


async def stow_publish_loop() -> None:
    """Publish STOW_POSITION at 10 Hz while ra_mode == 'stow'."""
    try:
        while get_ra_mode() == "stow" and ik_pos_pub is not None:
            ik_pos_pub.publish(STOW_POSITION)
            await asyncio.sleep(0.1)
    except asyncio.CancelledError:
        pass


def register_ik_pos_pub(pub: Publisher) -> None:
    global ik_pos_pub
    ik_pos_pub = pub


def register_tf_buffer(buffer: tf2_ros.Buffer) -> None:
    global tf_buffer
    tf_buffer = buffer


def get_ra_mode() -> str:
    with ra_mode_lock:
        return ra_mode


async def set_ra_mode(new_ra_mode: str) -> bool:
    global ra_mode, stow_task
    if new_ra_mode in ("ik-pos", "stow"):
        if not await call_ik_mode_service(IK_MODE_POSITION_CONTROL):
            return False
    elif new_ra_mode == "ik-vel":
        if not await call_ik_mode_service(IK_MODE_VELOCITY_CONTROL):
            return False

    with ra_mode_lock:
        ra_mode = new_ra_mode

    if new_ra_mode == "stow":
        if ik_pos_pub is not None:
            ik_pos_pub.publish(STOW_POSITION)
        if stow_task is None or stow_task.done():
            stow_task = asyncio.create_task(stow_publish_loop())
    else:
        if stow_task is not None and not stow_task.done():
            stow_task.cancel()
        stow_task = None

    return True


async def call_ik_mode_service(mode: int) -> bool:
    client = get_service_client(IkMode, "/ik_mode")
    request = IkMode.Request()
    request.mode = mode
    result = await call_service_async(client, request, timeout=5.0)
    return result.success if result else False


IK_MODE_POSITION_CONTROL = 0
IK_MODE_VELOCITY_CONTROL = 1
IK_MODE_TYPING = 2

STOW_POSITION_DEFAULTS = {
    "x": 1.124319,
    "y": 0.0,
    "z": 0.042229,
    "pitch": 0.072694,
    "roll": 0.0,
}
STOW_POSITION = IK()


def _quat_to_pitch_roll(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float]:
    sin_pitch = 2.0 * (qw * qy - qz * qx)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    pitch = math.asin(sin_pitch)
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    return pitch, roll


def _apply_stow_fields(values: dict) -> None:
    STOW_POSITION.pos.x = float(values["x"])
    STOW_POSITION.pos.y = float(values["y"])
    STOW_POSITION.pos.z = float(values["z"])
    STOW_POSITION.pitch = float(values.get("pitch", STOW_POSITION_DEFAULTS["pitch"]))
    STOW_POSITION.roll = float(values.get("roll", STOW_POSITION_DEFAULTS["roll"]))


def stow_position_dict() -> dict:
    return {
        "x": STOW_POSITION.pos.x,
        "y": STOW_POSITION.pos.y,
        "z": STOW_POSITION.pos.z,
        "pitch": STOW_POSITION.pitch,
        "roll": STOW_POSITION.roll,
    }


def load_stow_position() -> None:
    """Populate STOW_POSITION from config.db, seeding the default row on first boot."""
    stored = get_config(STOW_CONFIG_KEY)
    if stored is None:
        stored = STOW_POSITION_DEFAULTS
        set_config(STOW_CONFIG_KEY, stored)
    _apply_stow_fields(stored)


def update_stow_position(x: float, y: float, z: float, pitch: float, roll: float) -> dict:
    values = {"x": x, "y": y, "z": z, "pitch": pitch, "roll": roll}
    _apply_stow_fields(values)
    set_config(STOW_CONFIG_KEY, values)
    return stow_position_dict()


def capture_current_arm_pose() -> dict | None:
    """Look up arm_base_link -> arm_fk and return pose as {x,y,z,pitch,roll}, or None."""
    if tf_buffer is None:
        return None
    try:
        if not tf_buffer.can_transform("arm_base_link", "arm_fk", rclpy.time.Time()):
            return None
        arm_in_base = SE3.from_tf_tree(tf_buffer, "arm_fk", "arm_base_link")
    except (LookupException, ConnectivityException, ExtrapolationException):
        return None
    tx, ty, tz = arm_in_base.translation()
    qx, qy, qz, qw = arm_in_base.quat()
    pitch, roll = _quat_to_pitch_roll(float(qx), float(qy), float(qz), float(qw))
    return {
        "x": float(tx),
        "y": float(ty),
        "z": float(tz),
        "pitch": pitch,
        "roll": roll,
    }


# Seed defaults synchronously so the module is usable before ensure_initialized runs.
_apply_stow_fields(STOW_POSITION_DEFAULTS)


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
    "pusher",
    "gripper",
]

JOINT_SCALES = [
    -1.0,
    0.8,
    1.0,
    -1.0,
    1.0,
    0.4,
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
    return [names[i.value] for i in joints], [values[i.value] for i in joints]


def send_ra_controls(
    inputs: DeviceInputs, thr_pub: Publisher, ee_pos_pub: Publisher, ee_vel_pub: Publisher,
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
                    ik_pos_msg.pitch = 1.0 * simulated_axis(inputs.buttons, ControllerButton.RIGHT_TRIGGER, ControllerButton.LEFT_TRIGGER)
                    ik_pos_msg.roll = 1.0 * simulated_axis(inputs.buttons, ControllerButton.RIGHT_BUMPER, ControllerButton.LEFT_BUMPER)
                    ee_pos_pub.publish(ik_pos_msg)
                case "ik-vel":
                    ik_vel_msg = Twist()
                    ik_vel_msg.linear.x = (-1.0) * filter_input(safe_index(inputs.axes, ControllerAxis.LEFT_Y), deadzone=CONTROLLER_STICK_DEADZONE)
                    ik_vel_msg.linear.y = (-1.0) * filter_input(safe_index(inputs.axes, ControllerAxis.LEFT_X), deadzone=CONTROLLER_STICK_DEADZONE)
                    ik_vel_msg.linear.z = (-1.0) * filter_input(safe_index(inputs.axes, ControllerAxis.RIGHT_Y), deadzone=CONTROLLER_STICK_DEADZONE)
                    ik_vel_msg.angular.y = 1.0 * simulated_axis(inputs.buttons, ControllerButton.RIGHT_TRIGGER, ControllerButton.LEFT_TRIGGER)
                    ik_vel_msg.angular.x = 1.0 * simulated_axis(inputs.buttons, ControllerButton.RIGHT_BUMPER, ControllerButton.LEFT_BUMPER)
                    ee_vel_pub.publish(ik_vel_msg)

                    cam_throttle = filter_input(
                        simulated_axis(inputs.buttons, ControllerButton.Y, ControllerButton.A),
                        scale=JOINT_SCALES[Joint.CAM.value],
                    )
                    throttle_msg = Throttle()
                    throttle_msg.names = ["cam"]
                    throttle_msg.throttles = [cam_throttle]
                    thr_pub.publish(throttle_msg)
