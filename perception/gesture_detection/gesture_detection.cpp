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

        mDebugImgPub = create_publisher<sensor_msgs::msg::Image>("/gesture_detection/debug_image", 1);
        mImageSub = create_subscription<sensor_msgs::msg::Image>("/zed/left/image", rclcpp::QoS(1), [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
            imageCallback(msg);
        });
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

    auto GestureRecognitionNode::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void {
        if (mDebugImgPub->get_subscription_count() == 0) return;
        if (msg->encoding != sensor_msgs::image_encodings::BGRA8) {
            RCLCPP_WARN_STREAM_ONCE(get_logger(), "Debug overlay expects bgra8, got " << msg->encoding);
            return;
        }

        auto out = std::make_unique<sensor_msgs::msg::Image>(*msg);
        cv::Mat image{static_cast<int>(out->height), static_cast<int>(out->width), CV_8UC4, out->data.data(), out->step};

        double const scale = image.cols / 1280.0;
        int const thickness = std::max(1, static_cast<int>(std::lround(2 * scale)));
        cv::Scalar const good{0, 255, 0, 255}, bad{0, 0, 255, 255}, bone{255, 200, 0, 255}, text{255, 255, 255, 255};

        std::string status = "no body";
        if (mLatestBody && mLatestBody->detected) {
            msg::Body const& body = *mLatestBody;
            auto const& kp2d = body.keypoints_2d;
            auto visible = [&](std::size_t i) { return i < kp2d.size() && kp2d[i].x >= 0 && kp2d[i].y >= 0; };
            auto pixel = [&](std::size_t i) { return cv::Point{static_cast<int>(std::lround(kp2d[i].x)), static_cast<int>(std::lround(kp2d[i].y))}; };

            // BODY_18 skeleton (sl::BODY_18_PARTS)
            static constexpr std::array<std::pair<std::size_t, std::size_t>, 13> BONES{{
                    {0, 1},                   // nose - neck
                    {1, 2}, {2, 3}, {3, 4},   // right arm
                    {1, 5}, {5, 6}, {6, 7},   // left arm
                    {1, 8}, {8, 9}, {9, 10},  // right leg
                    {1, 11}, {11, 12}, {12, 13}, // left leg
            }};
            for (auto const& [a, b]: BONES) {
                if (visible(a) && visible(b)) cv::line(image, pixel(a), pixel(b), bone, thickness, cv::LINE_AA);
            }
            for (std::size_t i = 0; i < kp2d.size(); ++i) {
                if (!visible(i)) continue;
                cv::circle(image, pixel(i), 3 * thickness, jointValid(body, i) ? good : bad, cv::FILLED, cv::LINE_AA);
            }

            if (mPoseDetected) {
                status = "ASTRONAUT LOCKED";
            } else if (mHoldStart) {
                double const held = rclcpp::Time{body.header.stamp}.seconds() - *mHoldStart;
                status = std::format("T-pose {:.1f}/{:.1f}s", std::max(0.0, held), HOLD_SECONDS);
            } else {
                status = std::format("tracking id {} ({:.0f}%)", body.id, body.confidence);
            }
        }
        cv::putText(image, status, cv::Point{static_cast<int>(20 * scale), static_cast<int>(40 * scale)},
                    cv::FONT_HERSHEY_SIMPLEX, scale, text, thickness, cv::LINE_AA);

        mDebugImgPub->publish(std::move(out));
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
        mLatestBody = msg;

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
    }
} // namespace mrover

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::GestureRecognitionNode);