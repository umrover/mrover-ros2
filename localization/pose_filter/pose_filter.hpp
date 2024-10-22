#pragma once

#include "pch.hpp"

namespace mrover {

    class PoseFilter : public rclcpp::Node {

    private:

        auto ros_quat_to_eigen_quat(geometry_msgs::msg::Quaternion const& q) -> Eigen::Quaterniond;

        void pose_sub_callback(geometry_msgs::msg::Vector3Stamped::ConstSharedPtr const& msg);

        void correction_timer_callback();

        const rclcpp::Duration STEP = rclcpp::Duration::from_seconds(0.5);
        const rclcpp::Duration IMU_WATCHDOG_TIMEOUT = rclcpp::Duration::from_seconds(5.0);
        const rclcpp::Duration WINDOW{STEP * 2.5};
        
        static constexpr std::uint8_t FULL_CALIBRATION = 3;
        static constexpr float MIN_LINEAR_SPEED = 0.2, MAX_ANGULAR_SPEED = 0.1, MAX_ANGULAR_CHANGE = 0.2;

        rclcpp::TimerBase::SharedPtr imu_watchdog;
        rclcpp::TimerBase::SharedPtr correction_timer;
        
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_calib_sub;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr pose_sub;

        std::vector<geometry_msgs::msg::Twist> twists;
        std::optional<sensor_msgs::msg::Imu> current_imu_calib;

        std::optional<SE3d> last_pose_in_map;
        std::optional<builtin_interfaces::msg::Time> last_pose_time;
        std::optional<SO3d> correction_rotation;

        tf2_ros::Buffer tf_buffer{get_clock()};
        tf2_ros::TransformListener tf_listener{tf_buffer};
        tf2_ros::TransformBroadcaster tf_broadcaster{this};

        std::string rover_frame;
        std::string world_frame;

    public:
        PoseFilter();
    };

}