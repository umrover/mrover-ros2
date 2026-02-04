from backend.input import DeviceInputs, filter_input, simulated_axis
from backend.mappings import KeyboardButton
from mrover.msg import Throttle
from rclpy.publisher import Publisher

Y_SCALE = -1.0
Z_SCALE = 1.0


def send_mast_controls(keyboard: DeviceInputs, pub: Publisher) -> None:
    buttons = keyboard.buttons

    controller_y = filter_input(
        simulated_axis(buttons, KeyboardButton.W, KeyboardButton.S),
        scale=Y_SCALE,
    )
    controller_z = filter_input(
        simulated_axis(buttons, KeyboardButton.A, KeyboardButton.D),
        scale=Z_SCALE,
    )
    pub.publish(Throttle(names=["mast_gimbal_pitch", "mast_gimbal_yaw"], throttles=[controller_y, controller_z]))
