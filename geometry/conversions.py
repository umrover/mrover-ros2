from manifpy import SE3

import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Header


def from_position_orientation(tx=0, ty=0, tz=0, qx=0, qy=0, qz=0, qw=1) -> SE3:
    return SE3([tx, ty, tz], [qx, qy, qz, qw])


def to_tf_tree(
    tf_broadcaster: tf2_ros.TransformBroadcaster | tf2_ros.StaticTransformBroadcaster,
    se3: SE3,
    child_frame: str,
    parent_frame: str,
) -> None:
    tx, ty, tz = se3.translation()
    qx, qy, qz, qw = se3.quat()
    tf_broadcaster.sendTransform(
        TransformStamped(
            header=Header(frame_id=parent_frame),
            child_frame_id=child_frame,
            transform=Transform(
                translation=Vector3(x=tx, y=ty, z=tz),
                rotation=Quaternion(x=qx, y=qy, z=qz, w=qw),
            ),
        )
    )


def from_tf_tree(tf_buffer: tf2_ros.buffer, child_frame: str, parent_frame: str, time: Time = Time(), timeout: Duration = Duration()) -> SE3:
    se3, _ = from_tf_tree_with_time(tf_buffer, child_frame, parent_frame, time, timeout)
    return se3


def from_tf_tree_with_time(tf_buffer: tf2_ros.buffer, child_frame: str, parent_frame: str, time: Time = Time(), timeout: Duration = Duration()) -> tuple[SE3, Time]:
    tf = tf_buffer.lookup_transform(parent_frame, child_frame, time, timeout)
    translation = tf.transform.translation
    rotation = tf.transform.rotation
    tx, ty, tz = translation.x, translation.y, translation.z
    qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
    se3 = from_position_orientation(tx, ty, tz, qx, qy, qz, qw)
    return se3, tf.header.stamp
