import numpy as np

from lie import SO2
from state_machine.state import State
from . import stuck_recovery
from .approach_target import ApproachTargetState
from .context import Context
from coordinate_utils import is_high_cost_point


class LongRangeState(ApproachTargetState):
    """
    State for when the tag is seen only in the long range camera.
    Transitions:
    -If tag seen in ZED: ApproachPostState
    -Not seen in ZED and moved: LongRangeState
    -Don't see the tag in long range camera: SearchState
    -Stuck?
    """

    def on_exit(self, context: Context) -> None:
        pass

    def get_target_position(self, context: Context) -> np.ndarray | None:
        if context.course is None:
            return None

        current_waypoint = context.course.current_waypoint()
        if current_waypoint is None:
            return None

        if context.env.cost_map is None or not hasattr(context.env.cost_map, "data"):
            return None

        target = context.env.image_targets.query(context.course.image_target_name())
        if target is None:
            context.node.get_logger().info("Target not found in long range camera")
            return None

        rover_in_map = context.rover.get_pose_in_map()
        assert rover_in_map is not None

        rover_position = rover_in_map.translation()
        rover_direction = rover_in_map.rotation()[:, 0]

        bearing_to_tag = target.target.bearing
        # If you have not seen the tag in a while but are waiting until the expiration time is up,
        # keep going towards where it was last seen (the direction you are heading), don't use an old bearing value
        if target.hit_count <= 0:
            bearing_to_tag = 0

        direction_to_tag = SO2(bearing_to_tag).act(rover_direction[:2])

        distance = context.node.get_parameter("long_range.distance_ahead").value
        direction_to_tag = np.array([direction_to_tag[0], direction_to_tag[1], 0.0])

        tag_position = rover_position + direction_to_tag * distance
        while is_high_cost_point(point=tag_position, context=context):
            tag_position += direction_to_tag * distance
        context.node.get_logger().info(f"Long range target: {str(tag_position)}")
        return tag_position

    def next_state(self, context: Context, is_finished: bool) -> State:
        tag_position = context.env.current_target_pos()
        if tag_position is None:
            return self

        if context.rover.stuck:
            context.rover.previous_state = self
            return stuck_recovery.StuckRecoveryState()

        return ApproachTargetState()
