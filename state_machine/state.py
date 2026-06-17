from __future__ import annotations

from abc import ABC, abstractmethod
from typing import override


class State(ABC):
    """
    Abstract class that represents a state in the state machine.
    """

    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def on_enter(self, context) -> None:
        """
        Called exactly once when the state is entered. If any state needs to be initialized, do it here.
        :param context: The context object that is passed to the state machine.
        """
        raise NotImplementedError

    @abstractmethod
    def on_exit(self, context) -> None:
        """
        Called exactly once when the state is exited.
        No cleanup of internal state is necessary since this state will be destroyed.
        An example use case of this may be to write to an external log or send a service call to an external system.
        :param context: The context object that is passed to the state machine.
        """
        raise NotImplementedError

    @abstractmethod
    def on_loop(self, context) -> State:
        """
        Called repeatedly while the state is active.
        :param context: The context object that is passed to the state machine.
        :return: The next state to transition to. If the state should not change, return self.
        """
        raise NotImplementedError

    @override
    def __repr__(self) -> str:
        return self.__class__.__name__

    @override
    def __str__(self) -> str:
        return self.__repr__()

    @override
    def __eq__(self, other) -> bool:
        return self.__class__ == other.__class__

    @override
    def __hash__(self) -> int:
        return hash(self.__class__)

    @override
    def __ne__(self, other) -> bool:
        return not self.__eq__(other)


class ExitState(State):
    """
    State to be returned to signify that the state machine should exit.
    """

    @override
    def on_enter(self, context):
        pass

    @override
    def on_exit(self, context):
        pass

    @override
    def on_loop(self, context) -> State:
        return self
