from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pymap3d
import rclpy
from scipy import ndimage

import tf2_ros
from geometry_msgs.msg import Twist
from mrover.srv import MoveCostMap, DilateCostMap
from lie import SE3
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
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.time import Time
from rclpy.task import Future
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from state_machine.state import State
from std_msgs.msg import Bool, Header
from .drive import DriveController

NO_TAG: int = -1


@dataclass
class Rover:
    ctx: Context
    stuck: bool
    previous_state: State
    path_history: Path

    def get_pose_in_map(self) -> SE3 | None:
        try:
            return SE3.from_tf_tree(self.ctx.tf_buffer, self.ctx.rover_frame, self.ctx.world_frame)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            self.ctx.node.get_logger().warn(
                "Failed to get rover pose. Is localization running?", throttle_duration_sec=1
            )
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
    cost_map: CostMap

    arrived_at_target: bool = False
    arrived_at_waypoint: bool = False
    last_target_location: np.ndarray | None = None

    def get_target_position(self, frame: str) -> np.ndarray | None:
        """
        :param frame:   Target frame name. Could be for a tag, the hammer, or the water bottle.
        :return:        Pose of the target in the world frame if it exists and is not too old, otherwise None
        """
        try:
            target_pose, t = SE3.from_tf_tree_with_time(self.ctx.tf_buffer, frame, self.ctx.world_frame)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None

        now = self.ctx.node.get_clock().now()
        time = Time.from_msg(t)  # have to convert because time from message is a different type
        target_expiration_duration = Duration(seconds=self.ctx.node.get_parameter("target_expiration_duration").value)
        if now - time > target_expiration_duration:
            return None

        return target_pose.translation()

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

        increment_weight = self._context.node.get_parameter("image_targets.increment_weight").value
        decrement_weight = self._context.node.get_parameter("image_targets.decrement_weight").value
        # TODO(quintin): Seems like this was never used in 2024, might have been an oversight
        min_hits = self._context.node.get_parameter("image_targets.min_hits").value
        max_hits = self._context.node.get_parameter("image_targets.max_hits").value

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
                        seconds=self._context.node.get_parameter("target_expiration_duration").value
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
            seconds=self._context.node.get_parameter("long_range.bearing_expiration_duration").value
        ):
            return None
        return self._data[name]


class CostMap:
    """
    Context class to represent the costmap generated around the water bottle waypoint
    """

    data: np.ndarray
    resolution: int
    height: int
    width: int
    origin: np.ndarray


@dataclass
class Course:
    ctx: Context
    course_data: CourseMsg
    # Currently active waypoint
    waypoints: list[tuple[Waypoint, SE3]]
    waypoint_index: int = 0

    def increment_waypoint(self) -> int:
        self.waypoint_index = min(self.waypoint_index + 1, len(self.waypoints))
        return self.waypoint_index >= len(self.waypoints)

    def waypoint_pose(self, index: int) -> SE3:
        return self.waypoints[index][1]

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
        if self.ctx.env.image_targets.query(self.ctx.course.image_target_name()) is not None:
            self.ctx.node.get_logger().info("Tried to transition to long range")
            return long_range.LongRangeState()
        return None


def setup_course(ctx: Context, waypoints: list[tuple[Waypoint, SE3]]) -> Course:
    all_waypoint_info = []
    for index, (waypoint_info, waypoint_in_world) in enumerate(waypoints):
        all_waypoint_info.append(waypoint_info)

        # Either this or the lookup transform is broken
        SE3.to_tf_tree(ctx.tf_broadcaster, waypoint_in_world, f"course{index}", ctx.world_frame)
    # Make the course out of just the pure waypoint objects which is the 0th element in the tuple
    return Course(
        ctx=ctx,
        waypoints=waypoints,
        course_data=CourseMsg(waypoints=[waypoint for waypoint, _ in waypoints]),
    )


def convert_gps_to_cartesian(reference_point: np.ndarray, waypoint: GPSWaypoint) -> tuple[Waypoint, SE3]:
    """
    Converts a GPSWaypoint into a "Waypoint" used for publishing to the CourseService.
    """
    # Create a cartesian position based on GPS latitude and longitude
    x, y, _ = np.array(
        pymap3d.geodetic2enu(
            waypoint.latitude_degrees,
            waypoint.longitude_degrees,
            0,
            *reference_point,
            deg=True,
        )
    )
    # Zero the z-coordinate because even though the altitudes are set to zero,
    # Two points on a sphere are not going to have the same z-coordinate.
    # Navigation algorithms currently require all coordinates to have zero as the z-coordinate.
    return Waypoint(
        tag_id=waypoint.tag_id, type=waypoint.type, enable_costmap=waypoint.enable_costmap
    ), SE3.from_position_orientation(x, y)


def convert_cartesian_to_gps(reference_point: np.ndarray, coordinate: np.ndarray) -> GPSWaypoint:
    """
    Converts a coordinate to a GPSWaypoint (used for sending data back to basestation)
    """
    x, y, z = coordinate
    ref_lat, ref_lon, _ = reference_point
    lat, lon, _ = pymap3d.enu2geodetic(x, y, z, ref_lat, ref_lon, 0)
    return GPSWaypoint(
        latitude_degrees=lat,
        longitude_degrees=lon,
        type=WaypointType(val=WaypointType.NO_SEARCH),
    )


def convert_and_get_course(ctx: Context, reference_point: np.ndarray, request: EnableAuton.Request) -> Course:
    waypoints = [convert_gps_to_cartesian(reference_point, waypoint) for waypoint in request.waypoints]
    return setup_course(ctx, waypoints)


class Context:
    node: Node
    tf_buffer: tf2_ros.Buffer
    tf_listener: tf2_ros.TransformListener
    tf_broadcaster: tf2_ros.StaticTransformBroadcaster
    command_publisher: Publisher
    search_point_publisher: Publisher
    course_listener: Subscription
    stuck_listener: Subscription
    costmap_listener: Subscription
    path_history_publisher: Publisher

    # Use these as the primary interfaces in states
    course: Course | None
    rover: Rover
    drive: DriveController
    env: Environment
    disable_requested: bool

    # ROS parameters
    world_frame: str
    rover_frame: str

    def setup(self, node: Node):
        from .state import OffState

        self.node = node
        self.drive = DriveController(node)

        self.world_frame = node.get_parameter("world_frame").value
        self.rover_frame = node.get_parameter("rover_frame").value
        self.course = None
        self.rover = Rover(self, False, OffState(), Path(header=Header(frame_id=self.world_frame)))
        self.env = Environment(self, image_targets=ImageTargetsStore(self), cost_map=CostMap())
        self.disable_requested = False

        node.create_service(EnableAuton, "enable_auton", self.enable_auton)

        self.command_publisher = node.create_publisher(Twist, "nav_cmd_vel", 1)
        self.search_point_publisher = node.create_publisher(GPSPointList, "search_path", 1)
        self.path_history_publisher = node.create_publisher(Path, "ground_truth_path", 10)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(node)

        node.create_subscription(Bool, "nav_stuck", self.stuck_callback, 1)
        node.create_subscription(ImageTargets, "tags", self.image_targets_callback, 1)
        node.create_subscription(ImageTargets, "objects", self.image_targets_callback, 1)

        if node.get_parameter("costmap.custom_costmap").value:
            node.create_subscription(OccupancyGrid, "custom_costmap", self.costmap_callback, 1)
        else:
            node.create_subscription(OccupancyGrid, "costmap", self.costmap_callback, 1)
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, node)

        client_callback_group = MutuallyExclusiveCallbackGroup()

        self.move_cli = node.create_client(MoveCostMap, "move_cost_map", callback_group=client_callback_group)
        self.dilate_cli = node.create_client(DilateCostMap, "dilate_cost_map", callback_group=client_callback_group)

        if not node.get_parameter("costmap.custom_costmap").value:

            while not self.move_cli.wait_for_service(timeout_sec=1.0):
                node.get_logger().info("Waiting for move_cost_map service...")

            while not self.dilate_cli.wait_for_service(timeout_sec=1.0):
                node.get_logger().info("Waiting for dilate_cost service...")

    def enable_auton(self, request: EnableAuton.Request, response: EnableAuton.Response) -> EnableAuton.Response:
        self.node.get_logger().info("Received new course to navigate!")
        if request.enable:
            ref_point = np.array(
                [
                    self.node.get_parameter("ref_lat").value,
                    self.node.get_parameter("ref_lon").value,
                    self.node.get_parameter("ref_alt").value,
                ]
            )
            self.course = convert_and_get_course(self, ref_point, request)
        else:
            self.disable_requested = True
        response.success = True
        return response

    def stuck_callback(self, msg: Bool) -> None:
        self.rover.stuck = msg.data

    def image_targets_callback(self, tags: ImageTargets) -> None:
        self.env.image_targets.push_frame(tags.targets)

    def costmap_callback(self, msg: OccupancyGrid) -> None:
        """
        Callback function for the occupancy grid perception sends
        :param msg: Occupancy Grid representative of a 32m x 32m square area with origin at GNSS waypoint. Values are 0, 1, -1
        """
        unknown_cost = self.node.get_parameter("search.traversable_cost").value
        upsample_factor = 2
        filter_size = 5

        cost_map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width)).T.astype(np.float32)
        cost_map_data[cost_map_data == -1] = unknown_cost

        self.env.cost_map.origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.env.cost_map.resolution = msg.info.resolution / upsample_factor # meters/cell
        self.env.cost_map.height = msg.info.height * upsample_factor # cells
        self.env.cost_map.width = msg.info.width * upsample_factor # cells

        upsampled_cost_map = np.kron(cost_map_data, np.ones((upsample_factor,upsample_factor), np.float32))
        filtered_cost_map = ndimage.uniform_filter(upsampled_cost_map, filter_size, mode='nearest')


        self.env.cost_map.data = filtered_cost_map


        # array: known_free_cost
        # self.env.cost_map.data /= 100.0

    def move_costmap(self, course_name="base_link"):
        if self.node.get_parameter("costmap.custom_costmap").value:
            return
        self.node.get_logger().info(f"Requesting to move cost map to {course_name}")

        req = MoveCostMap.Request()
        req.course = course_name
        res: MoveCostMap.Response
        res = self.move_cli.call(req)

        if not res.success:
            self.node.get_logger().info("Move cost map failed")
        else:
            self.node.get_logger().info("Finished moving cost map")

    def dilate_cost(self, new_radius: float):
        if self.node.get_parameter("costmap.custom_costmap").value:
            return
        self.node.get_logger().info(f"Requesting to dilate cost to {new_radius}")

        req = DilateCostMap.Request()
        req.inflation_radius = new_radius
        res: DilateCostMap.Response
        res = self.dilate_cli.call(req)

        if not res.success:
            self.node.get_logger().info("Dilate cost map failed")
        else:
            self.node.get_logger().info("Finished dilating cost map")
