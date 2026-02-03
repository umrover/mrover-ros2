#pragma once

#include "pch.hpp"

namespace mrover {
    class Combiner final : public rclcpp::Node {
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSub;

        auto imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;

        // Object Bounding Boxes
        // Tags
        std::vector<std::vector<cv::Point2f>> mTagCorners;
        std::vector<int> mTagIds;
        rclcpp::Subscription<mrover::msg::TagBoundingBoxes>::SharedPtr mTagBoxesSub;

        // Mallet
        mrover::msg::ObjectBoundingBoxes mObjectBoxes;
        rclcpp::Subscription<mrover::msg::ObjectBoundingBoxes>::SharedPtr mObjectBoxesSub;

        // Message
        sensor_msgs::msg::Image mImage;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mImagePub;

        static auto getColor(std::string const& type) -> cv::Vec3b;

    public:
        explicit Combiner(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());
    };

} // namespace mrover
