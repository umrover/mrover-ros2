class ControllerAxis:
    LEFT_X = 0
    LEFT_Y = 1
    RIGHT_X = 2
    RIGHT_Y = 3
    LEFT_TRIGGER = 4
    RIGHT_TRIGGER = 5
    DPAD_X = 6
    DPAD_Y = 7

class ControllerButton:
    A = 0
    B = 1
    X = 2
    Y = 3
    LEFT_BUMPER = 4
    RIGHT_BUMPER = 5
    BACK = 6
    START = 7
    HOME = 8
    LEFT_STICK = 9
    RIGHT_STICK = 10
    DPAD_UP = 11
    DPAD_DOWN = 12
    DPAD_LEFT = 13
    DPAD_RIGHT = 14
    FORWARD = 15 # Not standard, but used in code

class JoystickAxis:
    FORWARD_BACK = 1
    LEFT_RIGHT = 0
    TWIST = 2
    THROTTLE = 3
    MICRO_FORWARD_BACK = 5 # Hat switch Y
    MICRO_LEFT_RIGHT = 4   # Hat switch X
