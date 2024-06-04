#!/usr/bin/env python3

import sys

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Point
from rclpy import Node
from state_machine.state_machine import StateMachine
from state_machine.state_publisher_server import StatePublisher
from .approach_target import ApproachTargetState
from .context import Context
from .long_range import LongRangeState
from .post_backup import PostBackupState
from .recovery import RecoveryState
from .search import SearchState
from .state import DoneState, OffState, off_check
from .waypoint import WaypointState


class Navigation(Node):
    state_machine: StateMachine
    context: Context
    state_machine_server: StatePublisher

    def __init__(self, context: Context):
        super().__init__("navigation")

        self.context = context

        self.get_logger().info("Navigation starting...")

        self.state_machine = StateMachine[Context](OffState(), "NavigationStateMachine", context)
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
        if rover_pose_in_map := self.context.rover.get_pose_in_map() is not None:
            x, y, _ = rover_pose_in_map.position
            self.context.rover.path_history.poses.append(
                PoseStamped(header=self.context.rover.path_history.header, pose=Pose(position=Point(x=x, y=y)))
            )
            self.context.path_history_publisher.publish(self.context.rover.path_history)


def main():
    rclpy.init(args=sys.argv)
    navigation = Navigation(Context())
    navigation.context.node = navigation
    rclpy.spin(navigation)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
