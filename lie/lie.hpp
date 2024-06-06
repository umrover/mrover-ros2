#pragma once

#include <rclcpp/time.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Geometry>

#include <manif/SE2.h>
#include <manif/SE3.h>
#include <manif/SO2.h>
#include <manif/SO3.h>

namespace mrover {

    using manif::SE3d, manif::SO3d, manif::SE3f, manif::SO3f, manif::SE2d, manif::SO2d, manif::SE2f, manif::SO2f;

    using R2d = Eigen::Vector2d;
    using R3d = Eigen::Vector3d;
    using S3d = Eigen::Quaterniond;

    using R2f = Eigen::Vector2f;
    using R3f = Eigen::Vector3f;
    using S3f = Eigen::Quaternionf;

    class SE3Conversions {
    public:
        static auto fromTf(geometry_msgs::msg::Transform const& transform) -> SE3d;

        static auto fromPose(geometry_msgs::msg::Pose const& pose) -> SE3d;

        [[nodiscard]] static auto toPose(SE3d const& tf) -> geometry_msgs::msg::Pose;

        [[nodiscard]] static auto toTransform(SE3d const& tf) -> geometry_msgs::msg::Transform;

        [[nodiscard]] static auto toPoseStamped(SE3d const& tf, std::string const& frame, rclcpp::Time const& time) -> geometry_msgs::msg::PoseStamped;

        [[nodiscard]] static auto toTransformStamped(SE3d const& tf, std::string const& childFrame, std::string const& parentFrame, rclcpp::Time const& time) -> geometry_msgs::msg::TransformStamped;

        /**
     * \brief           Pull the most recent transform or pose between two frames from the TF tree.
     *                  The second and third parameters are named for the transform interpretation.
     *                  Consider them named "a" and "b" respectively:
     *                  For a transform this is a rotation and translation, i.e. aToB.
     *                  For a pose this is a position and orientation, i.e. aInB.
     * \param buffer    ROS TF Buffer, make sure a listener is attached
     * \param fromFrame From (transform) or child (pose) frame
     * \param toFrame   To (transform) or parent (pose) frame
     * \param time      Time to query the transform at, default is the latest
     * \return          The transform or pose represented by an SE3 lie group element
     */
        [[nodiscard]] static auto fromTfTree(tf2_ros::Buffer const& buffer, std::string const& fromFrame, std::string const& toFrame, rclcpp::Time const& time = rclcpp::Time{}) -> SE3d;

        /**
     * \brief             Push a transform to the TF tree between two frames
     * \see               fromTfTree for more explanation of the frames
     * \param broadcaster ROS TF Broadcaster
     * \param fromFrame   From (transform) or child (pose) frame
     * \param toFrame     To (transform) or parent (pose) frame
     * \param transform   The transform or pose represented by an SE3 lie group element
     * \param time
     */
        static auto pushToTfTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& fromFrame, std::string const& toFrame, SE3d const& transform, rclcpp::Time const& time) -> void;
    };

    class SIM3 {
        using Transform = Eigen::Transform<double, 3, Eigen::Affine>;

        Transform mTransform = Transform::Identity();

    public:
        SIM3() = default;

        SIM3(SE3d const& se3, R3d const& scale);

        [[nodiscard]] auto matrix() const -> Eigen::Matrix4d;

        [[nodiscard]] auto position() const -> R3d;
    };

} // namespace mrover
