from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pymap3d
from manifpy import SE3

import tf2_ros
from geometry_msgs.msg import Twist
from mrover.msg import (
    Waypoint,
    GPSWaypoint,
    WaypointType,
    GPSPointList,
    Course as CourseMsg,
    ImageTarget,
    ImageTargets,
)
from mrover.srv import EnableAuton
from nav_msgs.msg import Path
from rclpy import Node
from rclpy.duration import Duration
from rclpy.publisher import Publisher
from rclpy.service import Service
from rclpy.subscription import Subscription
from rclpy.time import Time
from state_machine.state import State
from std_msgs.msg import Bool, Header
from .drive import DriveController

NO_TAG: int = -1

# TARGET_EXPIRATION_DURATION = Duration(60)

# LONG_RANGE_EXPIRATION_DURATION = rospy.Duration(rospy.get_param("long_range/time_threshold"))
# INCREMENT_WEIGHT = rospy.get_param("long_range/increment_weight")
# DECREMENT_WEIGHT = rospy.get_param("long_range/decrement_weight")
# MIN_HITS = rospy.get_param("long_range/min_hits")
# MAX_HITS = rospy.get_param("long_range/max_hits")
#
# REF_LAT = rospy.get_param("gps_linearization/reference_point_latitude")
# REF_LON = rospy.get_param("gps_linearization/reference_point_longitude")
# REF_ALT = rospy.get_param("gps_linearization/reference_point_altitude")
#
# tf_broadcaster: tf2_ros.StaticTransformBroadcaster = tf2_ros.StaticTransformBroadcaster()


@dataclass
class Rover:
    ctx: Context
    stuck: bool
    previous_state: State
    path_history: Path = Path(header=Header(frame_id="map"))
    driver: DriveController = DriveController()

    def get_pose_in_map(self) -> SE3 | None:
        try:
            return SE3.from_tf_tree(
                self.ctx.tf_buffer, parent_frame=self.ctx.world_frame, child_frame=self.ctx.rover_frame
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.ctx.node.get_logger().warn("Navigation failed to get rover pose. Is localization running?")
            return None

    def send_drive_command(self, twist: Twist) -> None:
        self.ctx.command_publisher.publish(twist)

    def send_drive_stop(self) -> None:
        self.send_drive_command(Twist())


@dataclass
class Environment:
    """
    Context class to represent the rover's environment
    Information such as locations of tags or obstacles
    """

    ctx: Context
    image_targets: ImageTargetsStore

    arrived_at_target: bool = False
    arrived_at_waypoint: bool = False
    last_target_location: np.ndarray | None = None

    def get_target_position(self, frame: str) -> np.ndarray | None:
        """
        :param frame:   Target frame name. Could be for a tag, the hammer, or the water bottle.
        :return:        Pose of the target in the world frame if it exists and is not too old, otherwise None
        """
        try:
            target_pose, time = SE3.from_tf_tree_with_time(
                self.ctx.tf_buffer, parent_frame=self.ctx.world_frame, child_frame=frame
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

        now = self.ctx.node.get_clock().now()
        target_expiration_duration = Duration(
            seconds=self.ctx.node.get_parameter("target_expiration_duration").get_parameter_value().double_value
        )
        if now - time > target_expiration_duration:
            return None

        return target_pose.position

    def current_target_pos(self) -> np.ndarray | None:
        assert self.ctx.course is not None

        match self.ctx.course.current_waypoint():
            case Waypoint(type=WaypointType(val=WaypointType.POST), tag_id=tag_id):
                return self.get_target_position(f"tag{tag_id}")
            case Waypoint(type=WaypointType(val=WaypointType.MALLET)):
                return self.get_target_position("hammer")
            case Waypoint(type=WaypointType(val=WaypointType.WATER_BOTTLE)):
                return self.get_target_position("bottle")
            case _:
                return None


class ImageTargetsStore:
    """
    Context class to represent the targets seen in the long range camera
    """

    @dataclass
    class TargetData:
        hit_count: int
        target: ImageTarget
        time: Time

    _data: dict[str, TargetData]
    _context: Context

    def __init__(self, context: Context) -> None:
        self._data = {}
        self._context = context

    def push_frame(self, targets: list[ImageTarget]) -> None:
        """
        Loops through our current list of our stored targets and checks if the new message includes each target or doesn't.
        If it does include it, we will increment our hit count for that target, store the new information, and reset the time we saw it.
        If it does not include it, we will decrement our hit count for that target, and if the hit count becomes zero, then we remove it from our stored list.
        If there are targets in the new message that we don't have stored, we will add it to our stored list.
        :param targets: A list of image targets sent by perception, which includes an id/name and bearing for each target in the list
        """
        now = self._context.node.get_clock().now()

        increment_weight = (
            self._context.node.get_parameter("image_targets/increment_weight").get_parameter_value().integer_value
        )
        decrement_weight = (
            self._context.node.get_parameter("image_targets/decrement_weight").get_parameter_value().integer_value
        )
        # TODO(quintin): Seems like this was never used in 2024, might have been an oversight
        min_hits = self._context.node.get_parameter("image_targets/min_hits").get_parameter_value().integer_value
        max_hits = self._context.node.get_parameter("image_targets/max_hits").get_parameter_value().integer_value

        # Update our current targets
        # Collect the iterator in to a list first since we will be modifying the dictionary
        target_names = {tag.name for tag in targets}
        for _, stored_tag in list(self._data.items()):
            # If we do see one of our targets in the new message, increment its hit count
            if stored_tag.target.name in target_names:
                stored_tag.hit_count += increment_weight
                if stored_tag.hit_count > max_hits:
                    stored_tag.hit_count = max_hits
            # If we do not see one of our targets in the new message, decrement its hit count
            else:
                stored_tag.hit_count -= decrement_weight
                if stored_tag.hit_count <= 0:
                    stored_tag.hit_count = 0
                    # If we haven't seen the target in a while, remove it from our list
                    time_difference = now - stored_tag.time
                    target_expiration_duration = Duration(
                        seconds=self._context.node.get_parameter("target_expiration_duration")
                        .get_parameter_value()
                        .double_value
                    )
                    if time_difference > target_expiration_duration:
                        del self._data[stored_tag.target.name]

        # Add or update seen targets
        for target in targets:
            # Keep hit count if already in the list, otherwise initialize
            hit_count = self._data[target.name].hit_count if target.name in self._data else increment_weight
            self._data[target.name] = self.TargetData(hit_count=hit_count, target=target, time=now)

    def query(self, name: str) -> TargetData | None:
        """
        :param name:    Image target name
        :return:        Image target if it exists and has been seen repeatedly recently, otherwise None
        """
        if not self._data:
            return None
        if name not in self._data:
            return None
        if self._context.node.get_clock().now() - self._data[name].time >= Duration(
            seconds=self._context.node.get_parameter("target_expiration_duration").get_parameter_value().double_value
        ):
            return None
        return self._data[name]


@dataclass
class Course:
    ctx: Context
    course_data: CourseMsg
    # Currently active waypoint
    waypoint_index: int = 0

    def increment_waypoint(self) -> None:
        self.waypoint_index += 1

    def waypoint_pose(self, index: int) -> SE3:
        waypoint_frame = f"course{index}"
        return SE3.from_tf_tree(self.ctx.tf_buffer, parent_frame="map", child_frame=waypoint_frame)

    def current_waypoint_pose_in_map(self) -> SE3:
        return self.waypoint_pose(self.waypoint_index)

    def current_waypoint(self) -> Waypoint | None:
        """
        :return: The currently active waypoint if we have an active course
        """
        if self.course_data is None:
            return None
        if self.waypoint_index >= len(self.course_data.waypoints):
            return None
        return self.course_data.waypoints[self.waypoint_index]

    def look_for_post(self) -> bool:
        """
        :return: Whether the currently active waypoint is a post (if it exists)
        """
        current_waypoint = self.current_waypoint()
        return current_waypoint is not None and current_waypoint.type.val == WaypointType.POST

    def look_for_object(self) -> bool:
        """
        :return: Whether the currently active waypoint is an object (if it exists).
                 Either the mallet or the water bottle.
        """
        current_waypoint = self.current_waypoint()
        return current_waypoint is not None and current_waypoint.type.val in {
            WaypointType.MALLET,
            WaypointType.WATER_BOTTLE,
        }

    def image_target_name(self) -> str:
        match self.current_waypoint():
            case Waypoint(tag_id=tag_id, type=WaypointType(val=WaypointType.POST)):
                return f"tag{tag_id}"
            case Waypoint(type=WaypointType(val=WaypointType.MALLET)):
                return "hammer"
            case Waypoint(type=WaypointType(val=WaypointType.WATER_BOTTLE)):
                return "bottle"
            case Waypoint(type=WaypointType(val=WaypointType.NO_SEARCH)):
                return ""
            case _:
                assert False

    def is_complete(self) -> bool:
        return self.waypoint_index == len(self.course_data.waypoints)

    def get_approach_state(self) -> State | None:
        """
        :return: One of the approach states (ApproachTargetState or LongRangeState)
                 if we are looking for a post or object, and we see it in one of the cameras (ZED or long range)
        """
        from . import long_range, approach_target

        # If we see the target in the ZED, go to ApproachTargetState
        if self.ctx.env.current_target_pos() is not None:
            return approach_target.ApproachTargetState()
        # If we see the target in the long range camera, go to LongRangeState
        assert self.ctx.course is not None
        if (
            self.ctx.course.image_target_name() != "bottle"
            and self.ctx.env.image_targets.query(self.ctx.course.image_target_name()) is not None
        ):
            return long_range.LongRangeState()
        return None


def setup_course(ctx: Context, waypoints: list[tuple[Waypoint, SE3]]) -> Course:
    all_waypoint_info = []
    for index, (waypoint_info, pose) in enumerate(waypoints):
        all_waypoint_info.append(waypoint_info)
        pose.publish_to_tf_tree(ctx.tf_broadcaster, "map", f"course{index}")
    # Make the course out of just the pure waypoint objects which is the 0th element in the tuple
    return Course(ctx=ctx, course_data=CourseMsg(waypoints=[waypoint for waypoint, _ in waypoints]))


def convert_gps_to_cartesian(reference_point: np.ndarray, waypoint: GPSWaypoint) -> tuple[Waypoint, SE3]:
    """
    Converts a GPSWaypoint into a "Waypoint" used for publishing to the CourseService.
    """
    # Create a cartesian position based on GPS latitude and longitude
    x, y, _ = np.array(
        pymap3d.geodetic2enu(waypoint.latitude_degrees, waypoint.longitude_degrees, 0, *reference_point, deg=True)
    )
    # Zero the z-coordinate because even though the altitudes are set to zero,
    # Two points on a sphere are not going to have the same z-coordinate.
    # Navigation algorithms currently require all coordinates to have zero as the z-coordinate.
    return Waypoint(tag_id=waypoint.tag_id, type=waypoint.type), SE3.from_position_orientation(x, y)


def convert_cartesian_to_gps(reference_point: np.ndarray, coordinate: np.ndarray) -> GPSWaypoint:
    """
    Converts a coordinate to a GPSWaypoint (used for sending data back to basestation)
    """
    x, y, z = coordinate
    ref_lat, ref_long, _ = reference_point
    lat, long, _ = pymap3d.enu2geodetic(x, y, z, ref_lat, ref_long, 0)
    return GPSWaypoint(latitude_degree=lat, longitude_degrees=long, type=WaypointType(val=WaypointType.NO_SEARCH))


def convert_and_get_course(ctx: Context, reference_point: np.ndarray, request: EnableAuton.Request) -> Course:
    waypoints = [convert_gps_to_cartesian(reference_point, waypoint) for waypoint in request.waypoints]
    return setup_course(ctx, waypoints)


class Context:
    node: Node
    tf_buffer: tf2_ros.Buffer
    tf_listener: tf2_ros.TransformListener
    tf_broadcaster: tf2_ros.TransformBroadcaster
    command_publisher: Publisher
    search_point_publisher: Publisher
    course_listener: Subscription
    stuck_listener: Subscription
    path_history_publisher: Publisher
    enable_auton_service: Service

    # Use these as the primary interfaces in states
    course: Course | None
    rover: Rover
    env: Environment
    disable_requested: bool

    # ROS parameters
    world_frame: str
    rover_frame: str

    def setup(self, node: Node):
        from .state import OffState

        self.node = node

        self.course = None
        self.rover = Rover(self, False, OffState(), Path())
        self.env = Environment(self, image_targets=ImageTargetsStore(self))
        self.disable_requested = False
        self.world_frame = node.get_parameter("world_frame").get_parameter_value().string_value
        self.rover_frame = node.get_parameter("rover_frame").get_parameter_value().string_value

        self.enable_auton_service = node.create_service(EnableAuton, "enable_auton", self.recv_enable_auton)

        self.command_publisher = node.create_publisher(Twist, "nav_cmd_vel", 1)
        self.search_point_publisher = node.create_publisher(GPSPointList, "search_path", 1)
        self.path_history_publisher = node.create_publisher(Path, "ground_truth_path", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(node)

        node.create_subscription(Bool, "nav_stuck", self.stuck_callback, 1)
        node.create_subscription(ImageTargets, "tags", self.image_targets_callback, 1)
        node.create_subscription(ImageTargets, "objects", self.image_targets_callback, 1)
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, node)

    def recv_enable_auton(self, request: EnableAuton.Request, response: EnableAuton.Response) -> EnableAuton.Response:
        ref_lat = self.node.get_parameter("gps_linearization/reference_point_latitude").get_parameter_value()
        ref_long = self.node.get_parameter("gps_linearization/reference_point_longitude").get_parameter_value()
        ref_alt = self.node.get_parameter("gps_linearization/reference_point_altitude").get_parameter_value()
        if request.enable:
            self.course = convert_and_get_course(self, np.array([ref_lat, ref_long, ref_alt]), request)
        else:
            self.disable_requested = True
        response.success = True
        return response

    def stuck_callback(self, msg: Bool) -> None:
        self.rover.stuck = msg.data

    def image_targets_callback(self, tags: ImageTargets) -> None:
        self.env.image_targets.push_frame(tags.targets)
