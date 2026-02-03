#pragma once

#include "pch.hpp"

namespace mrover {
    class Combiner final : public rclcpp::Node {
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSub;

        auto imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;

        // Object Bounding Boxes
        // Tags
        mrover::msg::ObjectBoundingBoxes mTagBoxes;
        rclcpp::Subscription<mrover::msg::ObjectBoundingBoxes>::SharedPtr mTagBoxesSub;

        // Mallet
        mrover::msg::ObjectBoundingBoxes mMalletBoxes;
        rclcpp::Subscription<mrover::msg::ObjectBoundingBoxes>::SharedPtr mMalletBoxesSub;

        // Bottle
        mrover::msg::ObjectBoundingBoxes mBottleBoxes;
        rclcpp::Subscription<mrover::msg::ObjectBoundingBoxes>::SharedPtr mBottleBoxesSub;

        // Message
        sensor_msgs::msg::Image mImage;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mImagePub;

    public:
        explicit Combiner(std::string const& name, rclcpp::NodeOptions const& options = rclcpp::NodeOptions());
    };

} // namespace mrover
