#pragma once

#include "mrover/action/detail/typing_position__struct.hpp"
#include "pch.hpp"

// #include "constants.h"
namespace mrover{
    class KeyboardTypingNode : public rclcpp::Node{
        private:
        // TypingPosition action client - this communicates with Nav
        using TypingPosition = mrover::action::TypingPosition;
        using GoalHandleTypingPosition = rclcpp_action::ClientGoalHandle<action::TypingPosition>;

        using PusherSrv = srv::Pusher;

        rclcpp_action::Client<action::TypingPosition>::SharedPtr mTypingClient;

        // Ik mode client
        rclcpp::Client<srv::IkMode>::SharedPtr mIkModeClient;

        void feedback_callback(GoalHandleTypingPosition::SharedPtr, const std::shared_ptr<const TypingPosition::Feedback> feedback);

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

        std::string current_key;

        double keyboard_roll;

        // Params
        int mMinCodeLength{}, mMaxCodeLength{};
        cv::Mat mCameraMatrix;
        cv::Mat mDistCoeffs;
        double mTagSize;
        Eigen::Vector3d mZKeyTransform;

        // transform broadcaster
        tf2_ros::Buffer tf_buffer{get_clock()};
        tf2_ros::TransformListener tf_listener{tf_buffer};
        tf2_ros::TransformBroadcaster tf_broadcaster{this};

        // transform from camera to end effector
        SE3d cam_to_gripper;

        // Define a board
        cv::Ptr<cv::aruco::Board> rover_board;

        // Sub to /finger_camera/image topic
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSub;

        int cnt = 0;

        LoopProfiler mLoopProfiler;

        // Can pub to any topic just make the name make sense
        rclcpp::Publisher<msg::KeyboardYaw>::SharedPtr mYawPub;
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
        bool filter_i0nitialized_ = false;

        // Change the function signature to accept vectors
        auto updateKalmanFilter(cv::Vec3d &tvec, cv::Vec3d &rvec) -> geometry_msgs::msg::Pose;

        // auto getKeyToCameraTransform(cv::Vec3d const& rvec,
        //                              cv::Vec3d const& tvec,
        //                              cv::Vec3d const& tag_offset_key) -> cv::Mat;


        auto sendIKCommand(float x, float y, float z, float pitch, float roll) -> void;

        auto yawCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;

        auto estimatePose(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> std::optional<pose_output>;

        auto align_arm() -> void;

        auto align_to_z() -> void;

        // Median Filter

        auto vectorMedianFilter(cv::Vec3d tvec, cv::Vec3d rvec) -> std::pair<cv::Vec3d, cv::Vec3d>;

        auto squaredEuclideanDistance(const cv::Vec3d& a, const cv::Vec3d& b) -> double;

        std::deque<cv::Vec3d> tvec_window;

        std::deque<cv::Vec3d> rvec_window;

        public:

        explicit KeyboardTypingNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());
    };
}

extern std::unordered_map<char, cv::Vec3d> keyboard_offset;