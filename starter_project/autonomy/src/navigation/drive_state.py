import numpy as np

from context import Context
from drive import get_drive_command
from state_machine.state import State
from tag_seek import TagSeekState

class DriveState(State):
    def on_enter(self, context) -> None:
        pass

    def on_exit(self, context) -> None:
        pass

    def on_loop(self, context) -> State:
        target = np.array([5.5, 2.0, 0.0])

        # TODO: get the rover's pose, if it doesn't exist stay in DriveState (return self)
        SE3_pose = context.rover.get_pose()
        if SE3_pose is None:
            return self
        
        # TODO: get the drive command and completion status based on target and pose
        # (HINT: use get_drive_command(), with completion_thresh set to 0.7 and turn_in_place_thresh set to 0.2)
        drive_command, completion_status = get_drive_command(
            target_pos=target, rover_pose=SE3_pose, completion_thresh=0.1, turn_in_place_thresh=0.2
        )

        # TODO: if we are finished getting to the target, go to TagSeekState
        if completion_status:
            return TagSeekState()
        
        # TODO: send the drive command to the rover
        context.rover.send_drive_command(drive_command)

        # TODO: tell state machine to stay in the DriveState by returning self
        return self
