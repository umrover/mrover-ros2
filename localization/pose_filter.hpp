#include "pch.hpp"

namespace mrover {

    class PoseFilter : public rclcpp::Node {

    private:

        auto rosQuaternionToEigenQuaternion(geometry_msgs::msg::Quaternion const& q) -> Eigen::Quaterniond;

        void pose_sub_callback(geometry_msgs::msg::Vector3StampedConstPtr const& msg);

        void correction_timer_callback();

        const rclcpp::Duration STEP(0, 500000000);
        const rclcpp::Duration WINDOW(2 + STEP.seconds() / 2);
        constexpr std::uint8_t FULL_CALIBRATION = 3;
        constexpr double IMU_WATCHDOG_TIMEOUT = 1.0;
        constexpr float MIN_LINEAR_SPEED = 0.2, MAX_ANGULAR_SPEED = 0.1, MAX_ANGULAR_CHANGE = 0.2;

        rclcpp::TimerBase::SharedPtr imu_watchdog;
        rclcpp::TimerBase::SharedPtr correction_timer;
        
        rclcpp::Publisher<nav_msgs::msg::Odometry> odometry_pub;
        rclcpp::Subscription<mrover::msg::CalibrationStatus> calibration_status_sub;
        rclcpp::Subscription<geometry_msgs::msg::Twist> twist_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu> imu_uncalib_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu> imu_calib_sub;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped> pose_sub;

        std::optional<mrover::msg::CalibrationStatus> calibration_status;
        std::vector<geometry_msgs::msg::Twist> twists;
        std::optional<sensor_msgs::msg::Imu> current_imu_uncalib;
        std::optional<sensor_msgs::msg::Imu> current_imu_calib;

        std::optional<SE3> last_pose_in_map;
        std::optional<rclcpp::Time> last_pose_time;
        std::optional<SO3> correction_rotation;

        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener{tf_buffer};
        tf2_ros::TransformBroadcaster tf_broadcaster;

        auto rover_frame;
        auto world_frame;

    public:
        PoseFilter();
    };

}