from geometry_msgs.msg import Twist

from context import Context
from state_machine.state import State
from state import DoneState, FailState


class TagSeekState(State):
    def on_enter(self, context) -> None:
        pass

    def on_exit(self, context) -> None:
        pass

    def on_loop(self, context) -> State:
        DISTANCE_TOLERANCE = 0.995
        ANUGLAR_TOLERANCE = 0.3
        # TODO: get the tag's location and properties (HINT: use get_fid_data() from context.env)

        # TODO: if we don't have a tag: go to the FailState

        # TODO: if we are within angular and distance tolerances: go to DoneState

        # TODO: figure out the Twist command to be applied to move the rover closer to the tag

        # TODO: send Twist command to rover

        # TODO: stay in the TagSeekState (with outcome "working")
