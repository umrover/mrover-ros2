"""LED state management - replaces esw/led.cpp functionality"""

from mrover.msg import LED
from backend.ros_manager import get_node


class LEDManager:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return

        self._initialized = True
        self.teleop_enabled = False
        self.nav_state = "OffState"
        self.current_led_color = "red"
        self.callbacks = []
        self.led_pub = None

    def _get_led_publisher(self):
        if self.led_pub is None:
            node = get_node()
            self.led_pub = node.create_publisher(LED, "/led", 1)
        return self.led_pub

    def set_teleop_enabled(self, enabled: bool):
        self.teleop_enabled = enabled
        self._update_led()

    def set_nav_state(self, state: str):
        self.nav_state = state
        self._update_led()

    def register_callback(self, callback):
        self.callbacks.append(callback)

    def _update_led(self):
        if self.teleop_enabled:
            new_color = "blue"
        elif self.nav_state == "DoneState":
            new_color = "blinking-green"
        else:
            new_color = "red"

        if new_color != self.current_led_color:
            self.current_led_color = new_color

            led_pub = self._get_led_publisher()
            led_msg = LED()

            if new_color == "red":
                led_msg.color = LED.RED
            elif new_color == "blue":
                led_msg.color = LED.BLUE
            elif new_color == "blinking-green":
                led_msg.color = LED.BLINKING_GREEN

            led_pub.publish(led_msg)

            for callback in self.callbacks:
                callback(new_color)


_manager = LEDManager()


def set_teleop_enabled(enabled: bool):
    _manager.set_teleop_enabled(enabled)


def set_nav_state(state: str):
    _manager.set_nav_state(state)


def register_led_callback(callback):
    _manager.register_callback(callback)
