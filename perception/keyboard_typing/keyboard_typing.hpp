#pragma once

#include "pch.hpp"
#include <memory>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_action/types.hpp>

// #include "constants.h"
namespace mrover{
    class KeyboardTypingNode : public rclcpp::Node{
        private:
        // TypingDeltas action client - this communicates with Nav
        using TypingDeltas = mrover::action::TypingDeltas;
        using GoalHandleTypingDeltas = rclcpp_action::ClientGoalHandle<TypingDeltas>;

        rclcpp_action::Client<TypingDeltas>::SharedPtr mTypingClient;

        void feedback_callback(GoalHandleTypingDeltas::SharedPtr, const std::shared_ptr<const TypingDeltas::Feedback> feedback);

        auto send_goal(float x_delta, float y_delta) -> bool;

        // TypingCode action server - this communicates with teleop
        using TypingCode = mrover::action::TypingCode;
        using GoalHandleTypingCode = rclcpp_action::ServerGoalHandle<TypingCode>;

        rclcpp_action::Server<TypingCode>::SharedPtr mTypingServer;

        auto handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const TypingCode::Goal> goal) -> rclcpp_action::GoalResponse;
        auto handle_cancel(const std::shared_ptr<GoalHandleTypingCode> goal_handle) -> rclcpp_action::CancelResponse;
        void handle_accepted(const std::shared_ptr<GoalHandleTypingCode> goal_handle);

        std::shared_ptr<GoalHandleTypingCode> mAcceptedGoalHandle = nullptr;

        // Store information for pose estimation
        struct pose_output {
            geometry_msgs::msg::Pose pose;
            double yaw;
        };

        // Params
        int mMinCodeLength{}, mMaxCodeLength{};

        // transform broadcaster
        tf2_ros::Buffer tf_buffer{get_clock()};
        tf2_ros::TransformListener tf_listener{tf_buffer};
        tf2_ros::TransformBroadcaster tf_broadcaster{this};

        // transform from camera to end effector
        SE3d gripper_to_cam;

        // Define a board
        cv::Ptr<cv::aruco::Board> rover_board;

        // Sub to /finger_camera/image topic
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSub;

        int cnt = 0;

        LoopProfiler mLoopProfiler;

        // Can pub to any topic just make the name make sense
        rclcpp::Publisher<msg::KeyboardYaw>::SharedPtr mCostMapPub;
        rclcpp::Publisher<msg::IK>::SharedPtr mIKPub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mIKVelPub;


        // Tag offsets
        std::unordered_map<int, cv::Vec3d> offset_map;

        // First position is rotation vector, second is translation vector
        // std::vector<cv::Vec3d> current_estimate;
        std::optional<SE3d> mCameraToKey = std::nullopt;
        bool mUpdatePoseEstimate = true;

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

        // auto getKeyToCameraTransform(cv::Vec3d const& rvec,
        //                              cv::Vec3d const& tvec,
        //                              cv::Vec3d const& tag_offset_key) -> cv::Mat;


        auto sendIKCommand(float x, float y, float z, float pitch, float roll) -> void;

        auto yawCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;

        auto estimatePose(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> std::optional<pose_output>;

        auto align_arm() -> void;

        public:

        explicit KeyboardTypingNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());
    };
}