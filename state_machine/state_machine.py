__author__ = "Ankith Udupa"

import time
from collections import defaultdict
from dataclasses import dataclass
from typing import DefaultDict, Callable, TypeVar, Generic

from rclpy.impl.rcutils_logger import RcutilsLogger
from state_machine.state import State


@dataclass
class TransitionRecord:
    time: float
    origin_state: str
    dest_state: str


ContextType = TypeVar("ContextType")


class StateMachine(Generic[ContextType]):
    current_state: State
    state_transitions: DefaultDict[type[State], set[type[State]]]
    transition_log: list[TransitionRecord]
    context: ContextType
    name: str
    off_lambda: Callable[[ContextType], bool] | None
    off_state: State | None
    logger: RcutilsLogger

    def __init__(
        self,
        initial_state: State,
        name: str,
        context: ContextType,
        logger: RcutilsLogger,
    ):
        self.current_state = initial_state
        self.state_transitions = defaultdict(set)
        self.state_transitions[type(self.current_state)] = set()
        self.transition_log: list[TransitionRecord] = []
        self.context = context
        self.name = name
        self.off_lambda = None
        self.off_state = None
        self.logger = logger

    def update(self):
        current_state = self.current_state
        self.logger.debug(f"{self.name} state machine, current state: {str(current_state)}")
        if self.off_lambda is not None and self.off_lambda(self.context) and self.off_state is not None:
            next_state = self.off_state
        else:
            # TODO: Make sure no exceptions
            next_state = current_state.on_loop(self.context)
            try:
                next_state = current_state.on_loop(self.context)
            except Exception as e:
                self.logger.warn(f"Error in {str(current_state)}: {e}")
                return self

        if type(next_state) not in self.state_transitions[type(current_state)]:
            raise Exception(f"Invalid transition from {current_state} to {next_state}")
        if type(next_state) is not type(current_state):
            # TODO: Make sure no exceptions
            try:
                self.logger.debug(f"{self.name} state machine, transitioning to {str(next_state)}")
                current_state.on_exit(self.context)
                self.transition_log.append(TransitionRecord(time.time(), str(current_state), str(next_state)))
                self.current_state = next_state
                self.current_state.on_enter(self.context)
            except Exception as e:
                self.logger.debug(f"Error in {str(current_state)}: {e}")

    def add_transition(self, state_from: State, state_to: State) -> None:
        self.state_transitions[type(state_from)].add(type(state_to))

    def add_transitions(self, state_from: State, states_to: list[State]) -> None:
        for state_to in states_to:
            self.add_transition(state_from, state_to)
        self.add_transition(state_from, state_from)
        if self.off_state is not None:
            self.add_transition(state_from, self.off_state)

    def configure_off_switch(self, off_state: State, off_lambda: Callable[[ContextType], bool]):
        if type(off_state) not in self.state_transitions:
            raise Exception("Attempted to configure an off state that does not exist")

        self.off_state = off_state
        self.off_lambda = off_lambda
        for _, to_states in self.state_transitions.items():
            to_states.add(type(off_state))
