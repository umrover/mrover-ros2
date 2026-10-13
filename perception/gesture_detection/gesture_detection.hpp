#pragma once

#include "pch.hpp"

#include <optional>

namespace mrover {
    class GestureRecognitionNode : public rclcpp::Node {
    private:
        static constexpr double HOLD_SECONDS = 5.0;                  // how long the pose must be held
        static constexpr double GRACE_SECONDS = 1.0;                 // dropouts shorter than this don't reset the timer
        static constexpr float MIN_JOINT_CONFIDENCE = 30.0f;         // joints below this (or NaN) are treated as unseen
        static constexpr double MAX_ELBOW_BEND_DEG = 45.0;           // 0 = perfectly straight arm
        static constexpr double MAX_ARM_TILT_DEG = 35.0;             // allowed deviation from perpendicular to the torso
        static constexpr double MAX_ARM_TO_SHOULDER_LINE_DEG = 50.0; // arms must point out along the shoulder line

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

    public:
        explicit GestureRecognitionNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions{});
    };
} // namespace mrover