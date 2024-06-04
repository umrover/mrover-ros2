from manifpy import SE3

import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
from rclpy.duration import Duration
from std_msgs.msg import Header


def to_tf_tree(
    tf_broadcaster: tf2_ros.TransformBroadcaster | tf2_ros.StaticTransformBroadcaster,
    se3: SE3,
    child_frame: str,
    parent_frame: str,
):
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


def from_tf_tree(tf_buffer: tf2_ros.buffer, child_frame: str, parent_frame: str, timeout: Duration) -> SE3:
    tf = tf_buffer.lookup_transform(parent_frame, child_frame, timeout)
    tx, ty, tz = tf.transform.translation
    qx, qy, qz, qw = tf.transform.rotation
    return SE3(tx, ty, tz, qx, qy, qz, qw)


def from_position_orientation(tx=0, ty=0, tz=0, qx=0, qy=0, qz=0, qw=1):
    return SE3(tx, ty, tz, qx, qy, qz, qw)
