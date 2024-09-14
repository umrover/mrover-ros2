from abc import ABC, abstractmethod
from typing import List, Optional

from state_machine.state import State
from geometry_msgs.msg import Twist

from context import Context


# class BaseState(smach.State, ABC):
#     """
#     Custom base state which handles termination cleanly via smach preemption.
#     """

#     context: Context

#     def __init__(
#         self,
#         context: Context,
#         add_outcomes: Optional[List[str]] = None,
#         add_input_keys: Optional[List[str]] = None,
#         add_output_keys: Optional[List[str]] = None,
#     ):
#         add_outcomes = add_outcomes or []
#         add_input_keys = add_input_keys or []
#         add_output_keys = add_output_keys or []
#         super().__init__(
#             add_outcomes + ["terminated"],
#             add_input_keys,
#             add_output_keys,
#         )
#         self.context = context

#     def execute(self, ud):
#         """
#         Override execute method to add logic for early termination.
#         Base classes should override evaluate instead of this!
#         :param ud:  State machine user data
#         :return:    Next state, 'terminated' if we want to quit early
#         """
#         if self.preempt_requested():
#             self.service_preempt()
#             return "terminated"
#         return self.evaluate(ud)

#     @abstractmethod
#     def evaluate(self, ud: smach.UserData) -> str:
#         """Override me instead of execute!"""
#         ...

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