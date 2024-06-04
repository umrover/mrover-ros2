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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from state_machine.state_machine import StateMachine
from state_machine.state_publisher_server import StatePublisher


class Navigation(Node):
    state_machine: StateMachine
    ctx: Context
    state_machine_server: StatePublisher

    def __init__(self, ctx: Context):
        super().__init__("navigation", allow_undeclared_parameters=True)

        self.ctx = ctx

        self.get_logger().info("Navigation starting...")

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

        update_rate = self.get_parameter("update_rate").get_parameter_value().double_value
        pub_path_rate = self.get_parameter("pub_path_rate").get_parameter_value().double_value
        self.create_timer(update_rate, self.state_machine.update)
        self.create_timer(pub_path_rate, self.publish_path)

        self.get_logger().info("Navigation started!")

    def publish_path(self):
        if (rover_pose_in_map := self.ctx.rover.get_pose_in_map()) is not None:
            x, y, _ = rover_pose_in_map.translation()
            self.ctx.rover.path_history.poses.append(
                PoseStamped(header=self.ctx.rover.path_history.header, pose=Pose(position=Point(x=x, y=y)))
            )
            self.ctx.path_history_publisher.publish(self.ctx.rover.path_history)


if __name__ == '__main__':
    rclpy.init(args=sys.argv)

    context = Context()
    navigation = Navigation(context)
    context.setup(navigation)

    try:
        rclpy.spin(navigation)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
