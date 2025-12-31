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
        // Store information for pose estimation
        struct pose_output {
            geometry_msgs::msg::Pose pose;
            double yaw;
        };

        // Define a board
        cv::Ptr<cv::aruco::Board> rover_board;

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
        
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        // Layout map (ID -> Bottom-Left Corner Position)
        std::map<int, cv::Vec3d> layout;

        auto outputToCSV(cv::Vec3d &tvec, cv::Vec3d &rvec) -> void;

        auto createRoverBoard() -> void;

        rclcpp::Time last_prediction_time_;
        bool filter_initialized_ = false;

        // Change the function signature to accept vectors
        auto updateKalmanFilter(cv::Vec3d &tvec, cv::Vec3d &rvec) -> geometry_msgs::msg::Pose;

        auto getKeyToCameraTransform(cv::Vec3d const& rvec,
                                     cv::Vec3d const& tvec,
                                     cv::Vec3d const& tag_offset_key) -> cv::Mat;


        auto sendIKCommand(float x, float y, float z, float pitch, float roll) -> void;

        auto yawCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;

        auto estimatePose(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> std::optional<pose_output>;

        public:
        explicit KeyboardTypingNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());
    };
}