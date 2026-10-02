import py_trees
from py_trees.common import Status
from navigation.context import Context

class CheckCondition(py_trees.behavior.Behavior):
    """Leaf node that checks a condition."""
    def __init__(self, name: str, success_condition: bool):
        super().__init__(name=name)
        self.success_condition = success_condition

    def update(self) -> Status:
        if self.success_condition:
            self.feedback_message = f"{self.name} passed."
            return Status.SUCCESS
        self.feedback_message = f"{self.name} failed."
        return Status.FAILURE

def build_tree(ctx : Context) -> py_trees.behavior.Behavior:
    """
    Constructs and returns the root node of the Behavior Tree.
    """

    backup_sel = py_trees.composites.Selector(
        name="Backup Fallback",
        memory=True,
        children=[
            CheckCondition(name="Target Doesn't Need to Backup", success_condition=True),
            CheckCondition(name="Initiate Backup", success_condition=True),
        ]
    )
    
    check_seq = py_trees.composites.Sequence(
        name="Condition Sequence Check",
        memory=True,
        children=[
            CheckCondition(name="Auton Running", success_condition=True),
            CheckCondition(name="Not Stuck", success_condition=True),
            backup_sel,
        ]
    )

    perform_actions = py_trees.composites.Sequence(
        name="Action Sequence",
        memory=True,
        children=[
            CheckCondition(name="Run Things", success_condition=True)
        ]
    )

    idle_seq = py_trees.composites.Sequence(
        name="Idle Sequence",
        memory=True,
        children=[
            CheckCondition(name="Idle", success_condition=True)
        ]
    )

    root = py_trees.composites.Sequence(
        name="Root Priority Selector",
        memory=False,
        children=[
            check_seq,
            perform_actions,
            idle_seq
        ]
    )

    return root