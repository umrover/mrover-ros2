#pragma once

#include "pch.hpp"
// #include "constants.h"

class KeyboardTypingNode : public rclcpp::Node{
    private:
    // Sub to /finger_camera/image topic
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSub;

    // Can pub to any topic just make the name make sense
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr mCostMapPub;

    auto estimatePose(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> geometry_msgs::msg::Pose;

    auto kalmanFilter(cv::Vec3d &tvec, cv::Vec3d &rvecs) -> void;

    auto yawCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;

    public:
    explicit KeyboardTypingNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());
};