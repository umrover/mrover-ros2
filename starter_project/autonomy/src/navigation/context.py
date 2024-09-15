from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Twist
from mrover.msg import StarterProjectTag

import sys
import os

sys.path.append(os.getcwd() + "/starter_project/autonomy/src")
from util.SE3 import SE3
from visualization_msgs.msg import Marker


@dataclass
class Rover:
    ctx: Context

    def get_pose(self) -> Optional[SE3]:
        # TODO: return the pose of the rover (or None if we don't have one (catch exception))
        pass

    def send_drive_command(self, twist: Twist):
        # TODO: send the Twist message to the rover
        pass

    def send_drive_stop(self):
        # TODO: tell the rover to stop
        pass


@dataclass
class Environment:
    """
    Context class to represent the rover's environment
    Information such as locations of fiducials or obstacles
    """

    ctx: Context
    fid_pos: Optional[StarterProjectTag]

    def receive_fid_data(self, message: StarterProjectTag):
        # TODO: handle incoming FID data messages here
        pass

    def get_fid_data(self) -> Optional[StarterProjectTag]:
        """
        Retrieves the last received message regarding fid pose
        if it exists, otherwise returns None
        """
        # TODO: return either None or your position message


class Context:
    node: Node
    tf_buffer: tf2_ros.Buffer
    tf_listener: tf2_ros.TransformListener
    vel_cmd_publisher: Publisher
    vis_publisher: Publisher
    fid_listener: Subscription

    # Use these as the primary interfaces in states
    rover: Rover
    env: Environment
    disable_requested: bool

    def setup(self, node: Node):
        self.node = node
        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.vel_cmd_publisher = node.create_publisher(Twist, "cmd_vel", 1)
        self.vis_publisher = node.create_publisher(Marker, "nav_vis", 1)
        self.rover = Rover(self)
        self.env = Environment(self, None)
        self.disable_requested = False

        self.fid_listener = node.create_subscription(StarterProjectTag, "/tag", self.env.receive_fid_data, 1)
