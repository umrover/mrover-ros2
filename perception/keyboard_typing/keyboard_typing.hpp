#pragma once

#include "mrover/msg/detail/ik__struct.hpp"
#include "pch.hpp"
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <mrover/msg/ik.hpp>
// #include "constants.h"
namespace mrover{
    class KeyboardTypingNode : public rclcpp::Node{
        private:
        // Sub to /finger_camera/image topic
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSub;

        LoopProfiler mLoopProfiler;

        // Can pub to any topic just make the name make sense
        rclcpp::Publisher<msg::KeyboardYaw>::SharedPtr mCostMapPub;
        rclcpp::Publisher<msg::IK>::SharedPtr mIKPub;

        // Tag offsets
        std::unordered_map<int, cv::Vec3d> offset_map;

        // First position is rotation vector, second is translation vector
        // std::vector<cv::Vec3d> current_estimate;

        // Filter that stores filtered pose
        cv::KalmanFilter kf;
        
        auto outputToCSV(cv::Vec3d &tvec, cv::Vec3d &rvec) -> void;

        rclcpp::Time last_prediction_time_;
        bool filter_initialized_ = false;

        // Change the function signature to accept vectors
        auto updateKalmanFilter(std::vector<cv::Vec3d> const& tvecs,
                                std::vector<cv::Vec3d> const& rvecs) -> geometry_msgs::msg::Pose;

        auto getGlobalCameraPosition(cv::Vec3d const& rvec, cv::Vec3d const& tvec, cv::Vec3d const& tag_offset_world) -> cv::Vec3d;

        auto estimatePose(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> std::optional<geometry_msgs::msg::Pose>;

        auto sendIKCommand(float x, float y, float z, float pitch, float roll) -> void;

        auto yawCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;

        public:
        explicit KeyboardTypingNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());
    };
}