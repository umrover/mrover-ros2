#!/usr/bin/env python3

import sys

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from rclpy import Node
from state_machine.state_machine import StateMachine
from state_machine.state_publisher_server import StatePublisher
from std_msgs.msg import Header
from .approach_target import ApproachTargetState
from .context import Context
from .long_range import LongRangeState
from .post_backup import PostBackupState
from .recovery import RecoveryState
from .search import SearchState
from .state import DoneState, OffState, off_check
from .water_bottle_search import WaterBottleSearchState
from .waypoint import WaypointState


class Navigation(Node):
    state_machine: StateMachine
    context: Context
    state_machine_server: StatePublisher

    def __init__(self, context: Context):
        super().__init__("navigation")

        self.get_logger().info("Navigation starting...")

        self.state_machine = StateMachine[Context](OffState(), "NavStateMachine", context)
        self.state_machine.add_transitions(
            ApproachTargetState(),
            [WaypointState(), SearchState(), WaterBottleSearchState(), RecoveryState(), DoneState()],
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
                WaterBottleSearchState(),
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
                WaterBottleSearchState(),
                LongRangeState(),
                SearchState(),
                RecoveryState(),
                DoneState(),
            ],
        )
        self.state_machine.add_transitions(
            LongRangeState(),
            [ApproachTargetState(), SearchState(), WaterBottleSearchState(), WaypointState(), RecoveryState()],
        )
        self.state_machine.add_transitions(OffState(), [WaypointState(), DoneState()])
        self.state_machine.add_transitions(
            WaterBottleSearchState(), [WaypointState(), RecoveryState(), ApproachTargetState(), LongRangeState()]
        )
        self.state_machine.configure_off_switch(OffState(), off_check)
        self.state_machine_server = StatePublisher(self, self.state_machine, "nav_structure", 1, "nav_state", 10)
        self.context = context

        self.create_timer(1, self.publish_path)
        self.create_timer(0.01, self.update)

        self.get_logger().info("Navigation started")

    def update(self):
        self.state_machine.update()

    def publish_path(self):
        self.context.rover.path_history.header = Header()
        self.context.rover.path_history.header.frame_id = "map"
        poses = []
        pose_stamped = PoseStamped()
        pose_stamped.header = Header()
        pose_stamped.header.frame_id = "map"
        rover_pose_in_map = self.context.rover.get_pose_in_map()
        if rover_pose_in_map is not None:
            point = Point(rover_pose_in_map.position[0], rover_pose_in_map.position[1], 0)
            quat = Quaternion(0, 0, 0, 1)
            pose_stamped.pose = Pose(point, quat)
            poses.append(pose_stamped)
            self.context.rover.path_history.poses = poses
            self.context.path_history_publisher.publish(self.context.rover.path_history)


def main():
    rclpy.init(args=sys.argv)
    navigation = Navigation(Context())
    rclpy.spin(navigation)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
