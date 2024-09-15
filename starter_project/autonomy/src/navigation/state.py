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
    
class OffState(State):
    def on_enter(self, context) -> None:
        pass

    def on_exit(self, context) -> None:
        pass

    def on_loop(self, context) -> State:
        return self
    
def off_check(context: Context) -> bool:
    """
    function that state machine will call to check if the rover is turned off
    """
    if context.disable_requested:
        context.disable_requested = False
        return True
    return False