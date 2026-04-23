from mrover.msg import LED
from backend.managers.ros import get_node


class LEDManager:
    instance = None

    def __new__(cls):
        if cls.instance is None:
            cls.instance = super().__new__(cls)
            cls.instance.initialized = False
        return cls.instance

    def __init__(self):
        if self.initialized:
            return

        self.initialized = True
        self.teleop_enabled = False
        self.nav_state = "OffState"
        self.current_led_color = "red"
        self.led_pub = None

    def get_led_publisher(self):
        if self.led_pub is None:
            node = get_node()
            self.led_pub = node.create_publisher(LED, "/led", 1)
            node.create_timer(0.5, self.publish_led)
        return self.led_pub

    def set_teleop_enabled(self, enabled: bool):
        self.teleop_enabled = enabled
        self.update_led()

    def set_nav_state(self, state: str):
        self.nav_state = state
        self.update_led()

    def update_led(self):
        if self.teleop_enabled:
            self.current_led_color = "blue"
        elif self.nav_state == "DoneState":
            self.current_led_color = "blinking-green"
        else:
            self.current_led_color = "red"
        self.publish_led()

    def publish_led(self):
        led_msg = LED()
        if self.current_led_color == "red":
            led_msg.color = LED.RED
        elif self.current_led_color == "blue":
            led_msg.color = LED.BLUE
        elif self.current_led_color == "blinking-green":
            led_msg.color = LED.BLINKING_GREEN
        self.get_led_publisher().publish(led_msg)


manager = LEDManager()


def set_teleop_enabled(enabled: bool):
    manager.set_teleop_enabled(enabled)


def set_nav_state(state: str):
    manager.set_nav_state(state)
