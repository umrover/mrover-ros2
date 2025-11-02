#pragma once

#include "pch.hpp"

class KeyboardTypingNode : public rclcpp::Node{
    private:
    // Sub to /finger_camera/image topic
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSub;

    // Can pub to any topic just make the name make sense
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr mCostMapPub;

    auto yawCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;

    public:
    explicit KeyboardTypingNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());
};