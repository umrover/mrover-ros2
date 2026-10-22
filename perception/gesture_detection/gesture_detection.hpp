#pragma once

#include "pch.hpp"

#include <optional>
#include <tf2_ros/tf2_ros/transform_broadcaster.hpp>

namespace mrover {
    class GestureRecognitionNode : public rclcpp::Node {
    private:
        double HOLD_SECONDS;                  // how long the pose must be held
        double GRACE_PERIOD;                 // dropouts shorter than this don't reset the timer
        float MIN_JOINT_CONFIDENCE;         // joints below this (or NaN) are treated as unseen
        double MAX_ELBOW_BEND_DEG;           // 0 = perfectly straight arm
        double MAX_ARM_TILT_DEG;             // allowed deviation from perpendicular to the torso
        double MAX_ARM_TO_SHOULDER_LINE_DEG; // arms must point out along the shoulder line

        // BODY_38 keypoint indices
        static constexpr std::size_t PELVIS = 0, NECK = 4;
        static constexpr std::size_t L_SHOULDER = 12, R_SHOULDER = 13;
        static constexpr std::size_t L_ELBOW = 14, R_ELBOW = 15;
        static constexpr std::size_t L_WRIST = 16, R_WRIST = 17;

        LoopProfiler mLoopProfiler;

        rclcpp::Subscription<msg::Body>::SharedPtr mBodySub;

        // T-pose hold timer state
        std::optional<double> mHoldStart; // time the current T-pose started, unset if not posing
        double mLastSeen{};               // last time a T-pose frame was seen
        bool mTriggered{};                // true once the hold has been reported

        auto poseCallback(msg::Body::ConstSharedPtr const& msg) -> void;

        auto isTPose(msg::Body const& body) -> bool;

        auto angleDeg(Eigen::Vector3d const& u, Eigen::Vector3d const& v) -> double;

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