from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Twist
from mrover.msg import StarterProjectTag

import sys
import os
sys.path.append(os.getcwd() + '/starter_project/autonomy/src')
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
        pass


class Context(Node):
    tf_buffer: tf2_ros.Buffer
    tf_listener: tf2_ros.TransformListener
    vel_cmd_publisher: rclpy.Publisher
    vis_publisher: rclpy.Publisher
    fid_listener: rclpy.Subscriber

    # Use these as the primary interfaces in states
    rover: Rover
    env: Environment

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.vel_cmd_publisher = self.create_publisher(Twist, "cmd_vel", queue_size=1)
        self.vis_publisher = self.create_publisher("nav_vis", Marker)
        self.rover = Rover(self)
        self.env = Environment(self, None)
        self.fid_listener = self.create_subscription(StarterProjectTag, "/tag", self.env.receive_fid_data)
