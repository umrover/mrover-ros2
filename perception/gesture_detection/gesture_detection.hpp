#pragma once

#include "pch.hpp"

#include <optional>
#include <tf2_ros/transform_broadcaster.hpp>

namespace mrover {
    class GestureRecognitionNode : public rclcpp::Node {
    private:
        double HOLD_SECONDS;                  // how long the pose must be held
        double GRACE_PERIOD;                 // dropouts shorter than this don't reset the timer
        float MIN_JOINT_CONFIDENCE;         // joints below this (or NaN) are treated as unseen
        double MAX_ELBOW_BEND_DEG;           // 0 = perfectly straight arm
        double MAX_ARM_TILT_DEG;             // allowed deviation from perpendicular to the torso
        double MAX_ARM_TO_SHOULDER_LINE_DEG; // arms must point out along the shoulder line

        // BODY_18 keypoint indices (sl::BODY_18_PARTS). There is no pelvis; we use the hip midpoint as the root.
        static constexpr std::size_t NECK = 1;
        static constexpr std::size_t R_SHOULDER = 2, R_ELBOW = 3, R_WRIST = 4;
        static constexpr std::size_t L_SHOULDER = 5, L_ELBOW = 6, L_WRIST = 7;
        static constexpr std::size_t R_HIP = 8, L_HIP = 11;

        LoopProfiler mLoopProfiler;

        rclcpp::Subscription<msg::Body>::SharedPtr mBodySub;

        // Debug overlay: draws the latest body's 2D keypoints on the left image
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mDebugImgPub;
        msg::Body::ConstSharedPtr mLatestBody;

        // T-pose hold timer state
        std::optional<double> mHoldStart; // time the current T-pose started, unset if not posing
        double mLastSeen{};               // last time a T-pose frame was seen
        bool mTriggered{};                // true once the hold has been reported

        bool mPoseDetected;

        auto poseCallback(msg::Body::ConstSharedPtr const& msg) -> void;

        auto isTPose(msg::Body const& body) -> bool;

        auto jointValid(msg::Body const& body, std::size_t i) const -> bool;

        auto jointsValid(msg::Body const& body, std::initializer_list<std::size_t> joints) const -> bool;

        auto imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;

        static auto angleDeg(Eigen::Vector3d const& u, Eigen::Vector3d const& v) -> double;

        struct Arm {
            std::size_t shoulder, elbow, wrist;
            Eigen::Vector3d outward;
        };

        // tf2 tree handling
        tf2_ros::Buffer tf_buffer{get_clock()};
        tf2_ros::TransformListener tf_listener{tf_buffer};
        tf2_ros::TransformBroadcaster tf_broadcaster{this};

    public:
        explicit GestureRecognitionNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions{});
    };
} // namespace mrover