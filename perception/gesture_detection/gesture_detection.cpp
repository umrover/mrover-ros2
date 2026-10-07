#include "gesture_detection.hpp"

namespace mrover{
    GestureRecognitionNode::GestureRecognitionNode(rclcpp::NodeOptions const& options) : rclcpp::Node("gesture_recognition_node", options), mLoopProfiler{get_logger()}
    {
        RCLCPP_INFO_STREAM(get_logger(), "GestureRecognitionNode starting up");

        mBodySub = create_subscription<msg::Body>("/zed/body", rclcpp::QoS(1), [this](msg::Body::ConstSharedPtr const& msg) {
            poseCallback(msg);
        });
    }

    auto GestureRecognitionNode::poseCallback(msg::Body::ConstSharedPtr const& msg) -> void {
        std::cout << "test" << std::endl;
        return;
    }
} // namespace mrover