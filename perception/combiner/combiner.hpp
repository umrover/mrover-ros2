#pragma once

#include "pch.hpp"

namespace mrover {
    class Combiner final : public rclcpp::Node {
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSub;

        auto imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;

        // CONSTANTS
        static constexpr std::size_t NUM_CLASSES = 3;
        static constexpr std::size_t WINDOW_SIZE = 5;
        static std::array<std::pair<std::string, cv::Vec3b>, NUM_CLASSES> const CLASSES;
        static constexpr std::size_t LINE_THICKNESS = 4;

        // Tags
        std::vector<std::vector<cv::Point2f>> mTagCorners;
        std::vector<int> mTagIds;
        rclcpp::Subscription<mrover::msg::TagBoundingBoxes>::SharedPtr mTagBoxesSub;

        // Mallet
        std::unordered_map<std::string, std::tuple<std::deque<cv::Rect>, std::deque<float>, std::deque<std::size_t>>> mObjectBoxes;
        rclcpp::Subscription<mrover::msg::ObjectBoundingBoxes>::SharedPtr mObjectBoxesSub;

        // Message
        sensor_msgs::msg::Image mImage;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mImagePub;

        static auto getColor(std::string const& type) -> cv::Vec3b;

    public:
        explicit Combiner(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());
    };

} // namespace mrover
