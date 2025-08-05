from __future__ import annotations
from typing import TextIO
from mrover.msg import StateMachineStructure, StateMachineTransition, StateMachineStateUpdate
from rclpy.node import Node
from rclpy.publisher import Publisher
from state_machine.state_machine import StateMachine
from state_machine.state import State


class StatePublisher:
    structure_publisher: Publisher
    state_publisher: Publisher
    last_state: State
    state_machine: StateMachine
    state_log: TextIO

    def __init__(
        self,
        node: Node,
        state_machine: StateMachine,
        structure_pub_topic: str,
        structure_update_rate_hz: float,
        state_pub_topic: str,
        state_update_rate_hz: float,
    ):
        self.state_machine = state_machine
        self.last_state = self.state_machine.current_state
        self.structure_publisher = node.create_publisher(StateMachineStructure, structure_pub_topic, 1)
        self.state_publisher = node.create_publisher(StateMachineStateUpdate, state_pub_topic, 1)
        node.create_timer(1 / structure_update_rate_hz, self.publish_structure)
        node.create_timer(1 / state_update_rate_hz, self.publish_state)
        self.state_log = open("./state_machine/state_log.txt", "w")

    def publish_structure(self) -> None:
        structure = StateMachineStructure()
        structure.machine_name = self.state_machine.name
        for origin, destinations in self.state_machine.state_transitions.items():
            transition = StateMachineTransition()
            transition.origin = origin.__name__
            transition.destinations = [dest.__name__ for dest in destinations]
            structure.transitions.append(transition)
        self.structure_publisher.publish(structure)

    def publish_state(self) -> None:
        current_state = self.state_machine.current_state
        if current_state != self.last_state:
            self.state_log.write(str(current_state) + "\n")
            self.last_state = current_state
        state = StateMachineStateUpdate(state=str(current_state), state_machine_name=self.state_machine.name)
        self.state_publisher.publish(state)
