#!/usr/bin/env python3

import sys

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Point
from navigation.approach_target import ApproachTargetState
from navigation.context import Context
from navigation.long_range import LongRangeState
from navigation.post_backup import PostBackupState
from navigation.recovery import RecoveryState
from navigation.search import SearchState
from navigation.state import DoneState, OffState, off_check
from navigation.waypoint import WaypointState
from rclpy import Parameter
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from state_machine.state_machine import StateMachine
from state_machine.state_publisher_server import StatePublisher


class Navigation(Node):
    state_machine: StateMachine
    ctx: Context
    state_machine_server: StatePublisher

    def __init__(self, ctx: Context) -> None:
        super().__init__("navigation")

        self.get_logger().info("Starting...")

        self.ctx = ctx

        self.declare_parameters(
            "",
            [
                ("update_rate", Parameter.Type.DOUBLE),
                ("pub_path_rate", Parameter.Type.DOUBLE),
                ("world_frame", Parameter.Type.STRING),
                ("rover_frame", Parameter.Type.STRING),
                ("ref_lat", Parameter.Type.DOUBLE),
                ("ref_lon", Parameter.Type.DOUBLE),
                ("ref_alt", Parameter.Type.DOUBLE),
                ("target_expiration_duration", Parameter.Type.DOUBLE),
                ("image_targets.increment_weight", Parameter.Type.INTEGER),
                ("image_targets.decrement_weight", Parameter.Type.INTEGER),
                ("image_targets.min_hits", Parameter.Type.INTEGER),
                ("image_targets.max_hits", Parameter.Type.INTEGER),
                ("drive.max_driving_effort", Parameter.Type.DOUBLE),
                ("drive.min_driving_effort", Parameter.Type.DOUBLE),
                ("drive.max_turning_effort", Parameter.Type.DOUBLE),
                ("drive.min_turning_effort", Parameter.Type.DOUBLE),
                ("drive.turning_p", Parameter.Type.DOUBLE),
                ("drive.driving_p", Parameter.Type.DOUBLE),
                ("drive.lookahead_distance", Parameter.Type.DOUBLE),
                ("waypoint.stop_threshold", Parameter.Type.DOUBLE),
                ("waypoint.drive_forward_threshold", Parameter.Type.DOUBLE),
                ("search.stop_threshold", Parameter.Type.DOUBLE),
                ("search.drive_forward_threshold", Parameter.Type.DOUBLE),
                ("search.coverage_radius", Parameter.Type.DOUBLE),
                ("search.segments_per_rotation", Parameter.Type.INTEGER),
                ("search.distance_between_spirals", Parameter.Type.DOUBLE),
                ("single_tag.stop_threshold", Parameter.Type.DOUBLE),
                ("single_tag.tag_stop_threshold", Parameter.Type.DOUBLE),
                ("single_tag.post_avoidance_multiplier", Parameter.Type.DOUBLE),
                ("single_tag.post_radius", Parameter.Type.DOUBLE),
            ],
        )

        self.state_machine = StateMachine[Context](OffState(), "NavigationStateMachine", ctx, self.get_logger())
        self.state_machine.add_transitions(
            ApproachTargetState(),
            [WaypointState(), SearchState(), RecoveryState(), DoneState()],
        )
        self.state_machine.add_transitions(PostBackupState(), [WaypointState(), RecoveryState()])
        self.state_machine.add_transitions(
            RecoveryState(),
            [
                WaypointState(),
                SearchState(),
                PostBackupState(),
                ApproachTargetState(),
                LongRangeState(),
            ],
        )
        self.state_machine.add_transitions(
            SearchState(),
            [ApproachTargetState(), LongRangeState(), WaypointState(), RecoveryState()],
        )
        self.state_machine.add_transitions(DoneState(), [WaypointState()])
        self.state_machine.add_transitions(
            WaypointState(),
            [
                PostBackupState(),
                ApproachTargetState(),
                LongRangeState(),
                SearchState(),
                RecoveryState(),
                DoneState(),
            ],
        )
        self.state_machine.add_transitions(
            LongRangeState(),
            [ApproachTargetState(), SearchState(), WaypointState(), RecoveryState()],
        )
        self.state_machine.add_transitions(OffState(), [WaypointState(), DoneState()])
        self.state_machine.configure_off_switch(OffState(), off_check)
        # TODO(quintin): Make the rates configurable as parameters
        self.state_machine_server = StatePublisher(self, self.state_machine, "nav_structure", 1, "nav_state", 10)

        update_rate = self.get_parameter("update_rate").value
        pub_path_rate = self.get_parameter("pub_path_rate").value
        self.create_timer(1 / update_rate, self.state_machine.update)
        self.create_timer(1 / pub_path_rate, self.publish_path)

        self.get_logger().info("Ready!")

    def publish_path(self) -> None:
        if (rover_pose_in_map := self.ctx.rover.get_pose_in_map()) is not None:
            x, y, _ = rover_pose_in_map.translation()
            self.ctx.rover.path_history.poses.append(
                PoseStamped(
                    header=self.ctx.rover.path_history.header,
                    pose=Pose(position=Point(x=x, y=y)),
                )
            )
            self.ctx.path_history_publisher.publish(self.ctx.rover.path_history)


if __name__ == "__main__":
    try:
        rclpy.init(args=sys.argv)

        context = Context()
        node = Navigation(context)
        context.setup(node)

        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
