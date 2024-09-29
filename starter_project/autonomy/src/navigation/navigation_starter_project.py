#!/usr/bin/env python3

import sys

# ros and state machine imports
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from state_machine.state_machine import StateMachine
from state_machine.state_publisher_server import StatePublisher

# navigation specific imports
from context import Context
from drive_state import DriveState
from state import DoneState, FailState
from tag_seek import TagSeekState


class Navigation(Node):
    state_machine: StateMachine
    ctx: Context
    state_machine_server: StatePublisher

    def __init__(self, ctx: Context):
        super().__init__("navigation")

        self.get_logger().info("Starting...")

        self.ctx = ctx

        self.state_machine = StateMachine[Context](DriveState(), "NavigationStateMachine", self.ctx, self.get_logger())

        # TODO: add DriveState and its transitions here

        # DoneState and its transitions
        self.state_machine.add_transitions(
            DoneState(),
            [DoneState()],
        )

        # FailState and its transitions
        self.state_machine.add_transitions(
            FailState(),
            [FailState()],
        )

        # TODO: add TagSeekState and its transitions here

        # self.state_machine_server = StatePublisher(self, self.state_machine, "nav_structure", 1, "nav_state", 10)

        self.create_timer(1 / 60, self.state_machine.update)


def main():
    try:
        # TODO: init a node called "navigation"

        # context and navigation objects
        context = Context()
        navigation = Navigation(context)
        context.setup(navigation)

        rclpy.spin(navigation)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    main()
