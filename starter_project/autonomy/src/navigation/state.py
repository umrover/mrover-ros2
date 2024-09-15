from abc import ABC, abstractmethod
from typing import List, Optional

from state_machine.state import State
from geometry_msgs.msg import Twist

from context import Context

class DoneState(State):
    def on_enter(self, context) -> None:
        pass

    def on_exit(self, context) -> None:
        pass

    def on_loop(self, context) -> State:
        # Stop rover
        cmd_vel = Twist()
        context.rover.send_drive_command(cmd_vel)
        return self