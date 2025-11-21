#pragma once

#include "pch.hpp"
// #include "constants.h"
namespace mrover{
    class KeyboardTypingNode : public rclcpp::Node{
        private:
        // Sub to /finger_camera/image topic
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSub;

        LoopProfiler mLoopProfiler;

        // Can pub to any topic just make the name make sense
        rclcpp::Publisher<msg::KeyboardYaw>::SharedPtr mCostMapPub;

        // Tag offsets
        std::unordered_map<int, cv::Vec3d> offset_map;

        // First position is rotation vector, second is translation vector
        // std::vector<cv::Vec3d> current_estimate;

        // Filter that stores filtered pose
        cv::KalmanFilter kf;

        rclcpp::Time last_prediction_time_;
        bool filter_initialized_ = false;

        // Change the function signature to accept vectors
        auto updateKalmanFilter(std::vector<cv::Vec3d> const& tvecs,
                                std::vector<cv::Vec3d> const& rvecs) -> geometry_msgs::msg::Pose;

        cv::Vec3d getGlobalCameraPosition(cv::Vec3d rvec, cv::Vec3d tvec, cv::Vec3d tag_offset_world);

        auto estimatePose(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> geometry_msgs::msg::Pose;

        auto yawCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;

        public:
        explicit KeyboardTypingNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());
    };
}