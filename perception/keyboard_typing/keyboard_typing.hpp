#pragma once

#include "pch.hpp"

constexpr double key_length = 0.01905;
constexpr double secondRowX = -0.0095;
constexpr double thirdRowX = -0.01345;
constexpr double fourthRowX = -0.009525;
constexpr double secondRowZ = 0.0007112;
constexpr double thirdRowZ = 0.0017272;
constexpr double fourthRowZ = 0.0048514;

inline std::unordered_map<char, cv::Vec3d> const keyboard_offset = {
        {'Z', cv::Vec3d{0, 0, 0}},
        {'X', cv::Vec3d{key_length, 0, 0}},
        {'C', cv::Vec3d{2 * key_length, 0, 0}},
        {'V', cv::Vec3d{3 * key_length, 0, 0}},
        {'B', cv::Vec3d{4 * key_length, 0, 0}},
        {'N', cv::Vec3d{5 * key_length, 0, 0}},
        {'M', cv::Vec3d{6 * key_length, 0, 0}},

        {'A', cv::Vec3d{secondRowX, key_length, secondRowZ}},
        {'S', cv::Vec3d{secondRowX + key_length, key_length, secondRowZ}},
        {'D', cv::Vec3d{secondRowX + 2 * key_length, key_length, secondRowZ}},
        {'F', cv::Vec3d{secondRowX + 3 * key_length, key_length, secondRowZ}},
        {'G', cv::Vec3d{secondRowX + 4 * key_length, key_length, secondRowZ}},
        {'H', cv::Vec3d{secondRowX + 5 * key_length, key_length, secondRowZ}},
        {'J', cv::Vec3d{secondRowX + 6 * key_length, key_length, secondRowZ}},
        {'K', cv::Vec3d{secondRowX + 7 * key_length, key_length, secondRowZ}},
        {'L', cv::Vec3d{secondRowX + 8 * key_length, key_length, secondRowZ}},

        {'Q', cv::Vec3d{thirdRowX, 2 * key_length, thirdRowZ}},
        {'W', cv::Vec3d{thirdRowX + key_length, 2 * key_length, thirdRowZ}},
        {'E', cv::Vec3d{thirdRowX + 2 * key_length, 2 * key_length, thirdRowZ}},
        {'R', cv::Vec3d{thirdRowX + 3 * key_length, 2 * key_length, thirdRowZ}},
        {'T', cv::Vec3d{thirdRowX + 4 * key_length, 2 * key_length, thirdRowZ}},
        {'Y', cv::Vec3d{thirdRowX + 5 * key_length, 2 * key_length, thirdRowZ}},
        {'U', cv::Vec3d{thirdRowX + 6 * key_length, 2 * key_length, thirdRowZ}},
        {'I', cv::Vec3d{thirdRowX + 7 * key_length, 2 * key_length, thirdRowZ}},
        {'O', cv::Vec3d{thirdRowX + 8 * key_length, 2 * key_length, thirdRowZ}},
        {'P', cv::Vec3d{thirdRowX + 9 * key_length, 2 * key_length, thirdRowZ}},

        // backspace
        {'-', cv::Vec3d{fourthRowX + 11 * key_length + 0.028575, 3 * key_length, fourthRowZ}}};

// #include "constants.h"
namespace mrover {
    class KeyboardTypingNode : public rclcpp::Node {
    private:
        // TypingPosition action client - this communicates with Nav
        using TypingPosition = mrover::action::TypingPosition;
        using GoalHandleTypingPosition = rclcpp_action::ClientGoalHandle<action::TypingPosition>;

        rclcpp_action::Client<action::TypingPosition>::SharedPtr mTypingClient;

        // Ik mode client
        rclcpp::Client<srv::IkMode>::SharedPtr mIkModeClient;

        void feedback_callback(GoalHandleTypingPosition::SharedPtr, std::shared_ptr<TypingPosition::Feedback const> const feedback);

        auto send_goal(float x_delta, float y_delta) -> bool;

        // TypingCode action server - this communicates with teleop
        using TypingCode = mrover::action::TypingCode;
        using GoalHandleTypingCode = rclcpp_action::ServerGoalHandle<TypingCode>;

        rclcpp_action::Server<TypingCode>::SharedPtr mTypingServer;

        auto handle_goal(rclcpp_action::GoalUUID const& uuid, std::shared_ptr<TypingCode::Goal const> goal) -> rclcpp_action::GoalResponse;
        auto handle_cancel(std::shared_ptr<GoalHandleTypingCode> const goal_handle) -> rclcpp_action::CancelResponse;
        void handle_accepted(std::shared_ptr<GoalHandleTypingCode> const goal_handle);

        std::shared_ptr<GoalHandleTypingCode> mAcceptedGoalHandle = nullptr;

        // Store information for pose estimation
        struct pose_output {
            geometry_msgs::msg::Pose pose;
            double yaw;
        };

        std::string current_key;

        double keyboard_roll;

        bool mAlignArm = false;

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
        std::map<int, cv::Vec3d> mTagLayout;

        auto outputToCSV(cv::Vec3d& tvec, cv::Vec3d& rvec) -> void;

        auto createRoverBoard() -> void;

        rclcpp::Time last_prediction_time_;
        bool filter_i0nitialized_ = false;

        // Change the function signature to accept vectors
        auto updateKalmanFilter(cv::Vec3d& tvec, cv::Vec3d& rvec) -> geometry_msgs::msg::Pose;

        auto sendIKCommand(float x, float y, float z, float pitch, float roll) -> void;

        auto yawCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;

        auto estimatePose(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> std::optional<pose_output>;

        auto align_arm() -> void;

        auto align_to_z() -> void;

        // Median Filter

        auto vectorMedianFilter(cv::Vec3d tvec, cv::Vec3d rvec) -> std::pair<cv::Vec3d, cv::Vec3d>;

        auto squaredEuclideanDistance(cv::Vec3d const& a, cv::Vec3d const& b) -> double;

        std::deque<cv::Vec3d> tvec_window;

        std::deque<cv::Vec3d> rvec_window;

    public:
        explicit KeyboardTypingNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());
    };
} // namespace mrover