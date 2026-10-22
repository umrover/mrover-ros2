#include "gesture_detection.hpp"

namespace mrover {
    GestureRecognitionNode::GestureRecognitionNode(rclcpp::NodeOptions const& options)
        : rclcpp::Node("gesture_recognition_node", options), mLoopProfiler{get_logger()} {
        RCLCPP_INFO_STREAM(get_logger(), "GestureRecognitionNode starting up");

        std::vector<ParameterWrapper> params{
                {"hold_seconds", HOLD_SECONDS, 5.0},
                {"grace_period", GRACE_PERIOD, 1.0},
                {"min_joint_confidence", MIN_JOINT_CONFIDENCE, 30.0},
                {"max_elbow_bend_deg", MAX_ELBOW_BEND_DEG, 45.0},
                {"max_arm_tilt_deg", MAX_ARM_TILT_DEG, 35.0},
                {"max_arm_to_shoulder_line_deg", MAX_ARM_TO_SHOULDER_LINE_DEG, 50.0},
                };

        ParameterWrapper::declareParameters(this, params);

        mBodySub = create_subscription<msg::Body>("/zed/body", rclcpp::QoS(1), [this](msg::Body::ConstSharedPtr const& msg) {
            poseCallback(msg);
        });
    }

    auto GestureRecognitionNode::angleDeg(Eigen::Vector3d const& u, Eigen::Vector3d const& v) -> double {
        double const cos = u.dot(v) / (u.norm() * v.norm());
        return std::acos(std::clamp(cos, -1.0, 1.0)) * 180.0 / std::numbers::pi;
    }

    auto GestureRecognitionNode::isTPose(msg::Body const& body) -> bool {
        if (!body.detected) return false;

        auto const& kp = body.keypoints;
        auto const& conf = body.keypoint_confidences;

        for (std::size_t i: {PELVIS, NECK, L_SHOULDER, R_SHOULDER, L_ELBOW, R_ELBOW, L_WRIST, R_WRIST}) {
            if (i >= kp.size() || i >= conf.size()) return false;
            if (!std::isfinite(conf[i]) || conf[i] < MIN_JOINT_CONFIDENCE) return false;
            if (!std::isfinite(kp[i].x) || !std::isfinite(kp[i].y) || !std::isfinite(kp[i].z)) return false;
        }

        auto point = [&](std::size_t i) { return Eigen::Vector3d{kp[i].x, kp[i].y, kp[i].z}; };

        Eigen::Vector3d const torsoUp = point(NECK) - point(PELVIS);
        Eigen::Vector3d const shoulderLine = point(L_SHOULDER) - point(R_SHOULDER); // right shoulder -> left

        for (Arm const& arm: {Arm{L_SHOULDER, L_ELBOW, L_WRIST, shoulderLine},
                                Arm{R_SHOULDER, R_ELBOW, R_WRIST, -shoulderLine}}) {
            Eigen::Vector3d const upperArm = point(arm.elbow) - point(arm.shoulder);
            Eigen::Vector3d const forearm = point(arm.wrist) - point(arm.elbow);
            Eigen::Vector3d const wholeArm = point(arm.wrist) - point(arm.shoulder);

            // Arm is straight
            if (!(angleDeg(upperArm, forearm) <= MAX_ELBOW_BEND_DEG)) return false;
            // Arm is horizontal relative to the torso
            if (!(std::abs(angleDeg(wholeArm, torsoUp) - 90.0) <= MAX_ARM_TILT_DEG)) return false;
            // Arm points out to the side, not forward or backward
            if (!(angleDeg(wholeArm, arm.outward) <= MAX_ARM_TO_SHOULDER_LINE_DEG)) return false;
        }
        return true;
    }

    auto GestureRecognitionNode::poseCallback(msg::Body::ConstSharedPtr const& msg) -> void {
        double const t = rclcpp::Time{msg->header.stamp}.seconds();

        if (isTPose(*msg)) {
            if (!mHoldStart) mHoldStart = t;
            mLastSeen = t;
            if (!mTriggered && t - *mHoldStart >= HOLD_SECONDS) {
                mTriggered = true;
                RCLCPP_INFO_STREAM(get_logger(), "T-pose detected (held for " << HOLD_SECONDS << " seconds)");
            }
        } else if (mHoldStart && t - mLastSeen > GRACE_PERIOD) {
            if (mTriggered) {
                RCLCPP_INFO_STREAM(get_logger(), "T-pose released");
            }
            mHoldStart.reset();
            mTriggered = false;
        }
    }
} // namespace mrover