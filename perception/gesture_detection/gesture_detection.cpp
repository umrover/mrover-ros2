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

        mPoseDetected = false;

        ParameterWrapper::declareParameters(this, params);

        mBodySub = create_subscription<msg::Body>("/zed/body", rclcpp::QoS(1), [this](msg::Body::ConstSharedPtr const& msg) {
            poseCallback(msg);
        });

        // Drawn by the camera client on top of the ZED stream
        mOverlayPub = create_publisher<msg::BodyOverlay>("/gesture_detection/body_overlay", rclcpp::QoS(1));
    }

    auto GestureRecognitionNode::publishOverlay(msg::Body const& body, double t) -> void {
        msg::BodyOverlay overlay;
        overlay.header = body.header;
        overlay.hold_required = static_cast<float>(HOLD_SECONDS);

        if (mPoseDetected) {
            overlay.state = msg::BodyOverlay::LOCKED;
        } else if (mHoldStart) {
            overlay.state = msg::BodyOverlay::HOLDING;
            overlay.hold_elapsed = static_cast<float>(std::max(0.0, t - *mHoldStart));
        } else if (body.detected) {
            overlay.state = msg::BodyOverlay::TRACKING;
        } else {
            overlay.state = msg::BodyOverlay::NO_BODY;
        }

        if (body.detected) {
            std::size_t const n = body.keypoints_2d.size();
            overlay.joints_u.reserve(n);
            overlay.joints_v.reserve(n);
            overlay.joints_confident.reserve(n);
            for (std::size_t i = 0; i < n; ++i) {
                overlay.joints_u.push_back(static_cast<float>(body.keypoints_2d[i].x));
                overlay.joints_v.push_back(static_cast<float>(body.keypoints_2d[i].y));
                overlay.joints_confident.push_back(jointValid(body, i));
            }
        }

        mOverlayPub->publish(overlay);
    }

    auto GestureRecognitionNode::angleDeg(Eigen::Vector3d const& u, Eigen::Vector3d const& v) -> double {
        double const cos = u.dot(v) / (u.norm() * v.norm());
        return std::acos(std::clamp(cos, -1.0, 1.0)) * 180.0 / std::numbers::pi;
    }

    auto GestureRecognitionNode::jointValid(msg::Body const& body, std::size_t i) const -> bool {
        auto const& kp = body.keypoints;
        auto const& conf = body.keypoint_confidences;
        return body.detected && i < kp.size() && i < conf.size() &&
               std::isfinite(conf[i]) && conf[i] >= MIN_JOINT_CONFIDENCE &&
               std::isfinite(kp[i].x) && std::isfinite(kp[i].y) && std::isfinite(kp[i].z);
    }

    auto GestureRecognitionNode::jointsValid(msg::Body const& body, std::initializer_list<std::size_t> joints) const -> bool {
        return std::ranges::all_of(joints, [&](std::size_t i) { return jointValid(body, i); });
    }

    auto GestureRecognitionNode::isTPose(msg::Body const& body) -> bool {
        if (!jointsValid(body, {L_HIP, R_HIP, NECK, L_SHOULDER, R_SHOULDER, L_ELBOW, R_ELBOW, L_WRIST, R_WRIST})) return false;

        auto const& kp = body.keypoints;
        auto point = [&](std::size_t i) { return Eigen::Vector3d{kp[i].x, kp[i].y, kp[i].z}; };

        Eigen::Vector3d const hipCenter = (point(L_HIP) + point(R_HIP)) / 2.0;
        Eigen::Vector3d const torsoUp = point(NECK) - hipCenter;
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

        if (!mPoseDetected) {
            if (isTPose(*msg)) {
                if (!mHoldStart) mHoldStart = t;
                mLastSeen = t;
                if (!mTriggered && t - *mHoldStart >= HOLD_SECONDS) {
                    mTriggered = true;
                    RCLCPP_INFO_STREAM(get_logger(), "T-pose detected (held for " << HOLD_SECONDS << " seconds)");
                    mPoseDetected = true;
                }
            } else if (mHoldStart && t - mLastSeen > GRACE_PERIOD) {
                if (mTriggered) {
                    RCLCPP_INFO_STREAM(get_logger(), "T-pose released");
                }
                mHoldStart.reset();
                mTriggered = false;
            }
        } else {
            // Publish the ZED body centroid in the camera frame, stamped with the image time.
            // Orientation is identity (aligned with the camera frame) and carries no meaning: only position is tracked.
            if (msg->detected) {
                SE3d const person{{msg->position.x, msg->position.y, msg->position.z}, Eigen::Quaterniond::Identity()};
                SE3Conversions::pushToTfTree(tf_broadcaster, "astronaut", msg->header.frame_id, person, msg->header.stamp);
            }
        }

        publishOverlay(*msg, t);
    }
} // namespace mrover

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::GestureRecognitionNode);