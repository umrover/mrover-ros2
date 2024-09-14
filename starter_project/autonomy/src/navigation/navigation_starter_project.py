#!/usr/bin/env python3

import signal
import sys
import threading

# ros and state machine imports
import rclpy
from rclpy.executors import ExternalShutdownException

from state_machine.state_machine import StateMachine
from state_machine.state_publisher_server import StatePublisher

# navigation specific imports
from context import Context
from drive_state import DriveState
from state import DoneState
from tag_seek import TagSeekState


class Navigation(threading.Thread):
    state_machine: StateMachine
    context: Context
    state_machine_server: StatePublisher

    def __init__(self, context: Context):
        super().__init__()
        self.name = "NavigationThread"
        self.state_machine = StateMachine(outcomes=["terminated"])
        self.context = context
        self.state_machine_server = StatePublisher(self, self.state_machine, "/SM_ROOT")
        self.state_machine = StateMachine[Context]

        # TODO: add DriveState and its transitions here

        # DoneState and its transitions
        self.state_machine.add_transitions(
            DoneState(),
            [DoneState()],
        )
        # TODO: add TagSeekState and its transitions here

    def run(self):
        self.state_machine.execute()

    def stop(self):
        self.sis.stop()
        # Requests current state to go into 'terminated' to cleanly exit state machine
        self.state_machine.request_preempt()
        # Wait for smach thread to terminate
        self.join()
        self.context.rover.send_drive_stop()


def main():
    try:
        # TODO: init a node called "navigation"

        # context and navigation objects
        context = Context()
        navigation = Navigation(context)

        rclpy.spin(navigation)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)

if __name__ == "__main__":
    main()
