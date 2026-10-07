#pragma once

#include "pch.hpp"

namespace mrover{
    class GestureRecognitionNode : public rclcpp::Node {
        private:

        LoopProfiler mLoopProfiler;

        rclcpp::Subscription<msg::Body>::SharedPtr mBodySub;

        void poseCallback(msg::Body::ConstSharedPtr const& msg);

        public:
        explicit GestureRecognitionNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());
    };
}