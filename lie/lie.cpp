#include "lie.hpp"

namespace mrover {

    SIM3::SIM3(SE3d const& se3, R3d const& scale) {
        mTransform.fromPositionOrientationScale(se3.translation(), se3.rotation(), scale);
    }

    auto SIM3::matrix() const -> Eigen::Matrix4d {
        return mTransform.matrix();
    }

    auto SIM3::position() const -> R3d {
        return mTransform.translation();
    }

    auto SE3Conversions::fromTfTree(tf2_ros::Buffer const& buffer, std::string const& fromFrame, std::string const& toFrame, rclcpp::Time const& time) -> SE3d {
        geometry_msgs::msg::TransformStamped transform = buffer.lookupTransform(toFrame, fromFrame, time);
        return fromTf(transform.transform);
    }

    auto SE3Conversions::pushToTfTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& fromFrame, std::string const& toFrame, SE3d const& transform, rclcpp::Time const& time) -> void {
        broadcaster.sendTransform(toTransformStamped(transform, fromFrame, toFrame, time));
    }

    auto SE3Conversions::fromTf(geometry_msgs::msg::Transform const& transform) -> SE3d {
        return {{transform.translation.x, transform.translation.y, transform.translation.z},
                Eigen::Quaterniond{transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z}};
    }

    auto SE3Conversions::fromPose(geometry_msgs::msg::Pose const& pose) -> SE3d {
        return {{pose.position.x, pose.position.y, pose.position.z},
                Eigen::Quaterniond{pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z}};
    }

    auto SE3Conversions::toPose(SE3d const& tf) -> geometry_msgs::msg::Pose {
        geometry_msgs::msg::Pose pose;
        pose.position.x = tf.x();
        pose.position.y = tf.y();
        pose.position.z = tf.z();
        SE3d::Quaternion const& q = tf.quat();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        return pose;
    }

    auto SE3Conversions::toTransform(SE3d const& tf) -> geometry_msgs::msg::Transform {
        geometry_msgs::msg::Transform transform;
        transform.translation.x = tf.x();
        transform.translation.y = tf.y();
        transform.translation.z = tf.z();
        SE3d::Quaternion const& q = tf.quat();
        transform.rotation.x = q.x();
        transform.rotation.y = q.y();
        transform.rotation.z = q.z();
        transform.rotation.w = q.w();
        return transform;
    }

    auto SE3Conversions::toPoseStamped(SE3d const& tf, std::string const& frame, rclcpp::Time const& time) -> geometry_msgs::msg::PoseStamped {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose = toPose(tf);
        pose.header.frame_id = frame;
        pose.header.stamp = time;
        return pose;
    }

    auto SE3Conversions::toTransformStamped(SE3d const& tf, std::string const& childFrame, std::string const& parentFrame, rclcpp::Time const& time) -> geometry_msgs::msg::TransformStamped {
        geometry_msgs::msg::TransformStamped transform;
        transform.transform = toTransform(tf);
        transform.header.frame_id = parentFrame;
        transform.header.stamp = time;
        transform.child_frame_id = childFrame;
        return transform;
    }

} // namespace mrover
