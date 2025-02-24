#pragma once

#include "pch.hpp"
#include <message_filters/subscriber.h>

namespace mrover {

    class PoseFilter : public rclcpp::Node {

    private:

        std::optional<mrover::msg::Heading> current_heading;
        std::optional<mrover::msg::FixStatus> current_heading_fix_status;

        double HEADING_THRESHOLD = (15 * (M_PI / 180));

        std::optional<double> averaged_heading_;
        std::optional<double> pose_callback_heading_;
        std::optional<double> correction_timer_heading_;

        auto ros_quat_to_eigen_quat(geometry_msgs::msg::Quaternion const& q) -> Eigen::Quaterniond;

        void pose_sub_callback(geometry_msgs::msg::Vector3Stamped::ConstSharedPtr const& linearized_pos_msg);

        void correction_timer_callback();

        void compare_and_select_heading();

        void heading_callback(const mrover::msg::Heading::ConstSharedPtr& heading_msg, const mrover::msg::FixStatus::ConstSharedPtr& fix_status_msg);

        const rclcpp::Duration STEP = rclcpp::Duration::from_seconds(0.5);
        const rclcpp::Duration IMU_WATCHDOG_TIMEOUT = rclcpp::Duration::from_seconds(1.0);
        const rclcpp::Duration WINDOW{STEP * 2.5};
        
        static constexpr std::uint8_t FULL_CALIBRATION = 3;
        static constexpr float MIN_LINEAR_SPEED = 0.2, MAX_ANGULAR_SPEED = 0.1, MAX_ANGULAR_CHANGE = 0.2;

        rclcpp::TimerBase::SharedPtr imu_watchdog;
        rclcpp::TimerBase::SharedPtr correction_timer;
        
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
        rclcpp::Subscription<mrover::msg::CalibrationStatus>::SharedPtr calibration_status_sub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr zed_imu_sub;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr pose_sub;

        message_filters::Subscriber<mrover::msg::Heading> heading_sub;
        message_filters::Subscriber<mrover::msg::FixStatus> heading_status_sub;

        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime
            <mrover::msg::Heading, mrover::msg::FixStatus>>> sync;

        std::optional<mrover::msg::CalibrationStatus> calibration_status;
        std::vector<geometry_msgs::msg::Twist> twists;
        std::optional<sensor_msgs::msg::Imu> current_imu;

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