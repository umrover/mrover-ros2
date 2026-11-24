from py_trees.blackboard import Client
from py_trees.common import Status, Access
from navigation.behavior_tree.behavior_tree import Leaf


class HoldingPick(Leaf):
    bb: Client

    def __init__(self, name: str = "Holding Pick?") -> None:
        super().__init__(name=name)

    def setup(self, **kwargs) -> None:
        super().setup(**kwargs)

        self.bb = self.attach_blackboard_client()
        # TODO: key name and writer not confirmed yet
        self.bb.register_key(key="holding_pick", access=Access.READ)

    def update(self) -> Status:
        return Status.SUCCESS if self.bb.holding_pick else Status.FAILURE


class DropPick(Leaf):
    def __init__(self, name: str = "Drop Pick") -> None:
        super().__init__(name=name)
