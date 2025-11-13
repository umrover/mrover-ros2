#!/usr/bin/env python3

import sys

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Point
from navigation.approach_target import ApproachTargetState
from navigation.context import Context
from navigation.long_range import LongRangeState
from navigation.backup import BackupState
from navigation.stuck_recovery import StuckRecoveryState
from navigation.costmap_search import CostmapSearchState
from navigation.state import DoneState, OffState, off_check
from navigation.waypoint import WaypointState
from nav_msgs.msg import Path
from rclpy import Parameter
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor
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
                # General
                ("update_rate", Parameter.Type.DOUBLE),
                ("pub_path_rate", Parameter.Type.DOUBLE),
                ("path_hist_size", Parameter.Type.INTEGER),
                ("display_markers", Parameter.Type.BOOL),
                ("world_frame", Parameter.Type.STRING),
                ("rover_frame", Parameter.Type.STRING),
                ("ref_lat", Parameter.Type.DOUBLE),
                ("ref_lon", Parameter.Type.DOUBLE),
                ("ref_alt", Parameter.Type.DOUBLE),
                ("target_expiration_duration", Parameter.Type.DOUBLE),
                # Costmap
                ("costmap.custom_costmap", Parameter.Type.BOOL),
                ("costmap.use_costmap", Parameter.Type.BOOL),
                ("costmap.costmap_thresh", Parameter.Type.DOUBLE),
                ("costmap.initial_inflation_radius", Parameter.Type.DOUBLE),
                # Backup
                ("backup.stop_threshold", Parameter.Type.DOUBLE),
                ("backup.drive_forward_threshold", Parameter.Type.DOUBLE),
                ("backup.backup_distance", Parameter.Type.DOUBLE),
                ("backup.wait_time", Parameter.Type.DOUBLE),
                # Drive
                ("drive.max_driving_effort", Parameter.Type.DOUBLE),
                ("drive.min_driving_effort", Parameter.Type.DOUBLE),
                ("drive.max_turning_effort", Parameter.Type.DOUBLE),
                ("drive.min_turning_effort", Parameter.Type.DOUBLE),
                ("drive.turning_p", Parameter.Type.DOUBLE),
                ("drive.driving_p", Parameter.Type.DOUBLE),
                ("drive.lookahead_distance", Parameter.Type.DOUBLE),
                # Waypoint
                ("waypoint.stop_threshold", Parameter.Type.DOUBLE),
                ("waypoint.drive_forward_threshold", Parameter.Type.DOUBLE),
                ("waypoint.no_search_wait_time", Parameter.Type.DOUBLE),
                # Long Range
                ("long_range.distance_ahead", Parameter.Type.DOUBLE),
                ("long_range.bearing_expiration_duration", Parameter.Type.DOUBLE),
                # Search
                ("search.stop_threshold", Parameter.Type.DOUBLE),
                ("search.drive_forward_threshold", Parameter.Type.DOUBLE),
                ("search.coverage_radius", Parameter.Type.DOUBLE),
                ("search.segments_per_rotation", Parameter.Type.INTEGER),
                ("search.distance_between_spirals", Parameter.Type.DOUBLE),
                ("search.max_segment_length", Parameter.Type.DOUBLE),
                ("search.traversable_cost", Parameter.Type.DOUBLE),
                ("search.update_delay", Parameter.Type.DOUBLE),
                ("search.safe_approach_distance", Parameter.Type.DOUBLE),
                ("search.angle_thresh", Parameter.Type.DOUBLE),
                ("search.distance_threshold", Parameter.Type.DOUBLE),
                # Image Targets
                ("image_targets.increment_weight", Parameter.Type.INTEGER),
                ("image_targets.decrement_weight", Parameter.Type.INTEGER),
                ("image_targets.min_hits", Parameter.Type.INTEGER),
                ("image_targets.max_hits", Parameter.Type.INTEGER),
                # Single Tag
                ("single_tag.stop_threshold", Parameter.Type.DOUBLE),
                ("single_tag.tag_stop_threshold", Parameter.Type.DOUBLE),
                ("single_tag.post_avoidance_multiplier", Parameter.Type.DOUBLE),
                ("single_tag.post_radius", Parameter.Type.DOUBLE),
                # Recovery
                ("recovery.stop_threshold", Parameter.Type.DOUBLE),
                ("recovery.drive_forward_threshold", Parameter.Type.DOUBLE),
                ("recovery.recovery_distance", Parameter.Type.DOUBLE),
                ("recovery.give_up_time", Parameter.Type.DOUBLE),
            ],
        )

        self.state_machine = StateMachine[Context](OffState(), "NavigationStateMachine", ctx, self.get_logger())
        self.state_machine.add_transitions(
            ApproachTargetState(),
            [
                WaypointState(),
                CostmapSearchState(),
                StuckRecoveryState(),
                BackupState(),
                DoneState(),
            ],
        )
        self.state_machine.add_transitions(BackupState(), [WaypointState(), DoneState()])
        self.state_machine.add_transitions(
            StuckRecoveryState(),
            [
                WaypointState(),
                CostmapSearchState(),
                BackupState(),
                ApproachTargetState(),
                LongRangeState(),
            ],
        )
        self.state_machine.add_transitions(DoneState(), [WaypointState()])
        self.state_machine.add_transitions(
            WaypointState(),
            [
                BackupState(),
                ApproachTargetState(),
                LongRangeState(),
                CostmapSearchState(),
                StuckRecoveryState(),
                DoneState(),
            ],
        )
        self.state_machine.add_transitions(
            LongRangeState(),
            [
                ApproachTargetState(),
                CostmapSearchState(),
                WaypointState(),
                StuckRecoveryState(),
            ],
        )
        self.state_machine.add_transitions(
            CostmapSearchState(), [WaypointState(), StuckRecoveryState(), ApproachTargetState(), LongRangeState()]
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

        self.HIST_SIZE = self.get_parameter("path_hist_size").value

    def publish_path(self) -> None:
        if (rover_pose_in_map := self.ctx.rover.get_pose_in_map()) is not None:
            x, y, z = rover_pose_in_map.translation()
            roverPoseStamped = PoseStamped(
                header=self.ctx.rover.path_history.header, pose=Pose(position=Point(x=x, y=y, z=z))
            )
            lastRoverPosition: Point | None = (
                None
                if len(self.ctx.rover.path_history.poses) == 0
                else self.ctx.rover.path_history.poses[-1].pose.position
            )

            if lastRoverPosition is None:
                self.ctx.rover.path_history.poses.append(roverPoseStamped)
            elif (
                lastRoverPosition is not None and (x - lastRoverPosition.x) ** 2 + (y - lastRoverPosition.y) ** 2
            ) ** 0.5 > 0.15:
                if len(self.ctx.rover.path_history.poses) > self.HIST_SIZE:
                    self.ctx.rover.path_history.poses.pop(0)
                self.ctx.rover.path_history.poses.append(roverPoseStamped)

            self.ctx.path_history_publisher.publish(self.ctx.rover.path_history)


if __name__ == "__main__":
    try:
        rclpy.init(args=sys.argv)

        context = Context()
        node = Navigation(context)
        context.setup(node)

        exec = SingleThreadedExecutor()
        context.exec = exec
        exec.add_node(node)
        exec.spin()

        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
