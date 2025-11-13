#pragma once

#include "pch.hpp"
#include <unordered_map>
// #include "constants.h"
namespace mrover{
    class KeyboardTypingNode final : public rclcpp::Node{
        private:
        struct Tag{
            int id = -1;
            int hitCount = 0;
            cv::Point2f center{};
        };

        // Sub to /finger_camera/image topic
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSub;

        // Can pub to any topic just make the name make sense
        rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr mQuaternionPub;

        // Needed for tag detection
        cv::Ptr<cv::aruco::Dictionary> mTagDictionary;
        std::vector<std::vector<cv::Point2f>> mTagCorners;
        std::vector<int> mTagIds;

        // TF tree
        // tf2_ros::Buffer mTfBuffer{get_clock()};
        // tf2_ros::TransformListener mTfListener{mTfBuffer};

        // Other vars
        Tag mSelectedTag;
        int mHitCount = 0;
        std::unordered_map<int, Tag> mTagMap;

        auto poseEstimation(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> geometry_msgs::msg::Quaternion;

        auto kalmanFilter(geometry_msgs::msg::Quaternion const& msg) -> geometry_msgs::msg::Quaternion;

        auto yawCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;

        auto findTagsInImage(cv::Mat const& image) -> void;

        public:
        explicit KeyboardTypingNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());
    };
}