#pragma once

#include "pch.hpp"

namespace mrover {

    class HeadingFilter : public rclcpp::Node {

    private:

        void correct(double heading_correction_delta_meas, double heading_correction_delta_noise);
        void predict(double process_noise);

        // callbacks
        void sync_rtk_heading_callback(const mrover::msg::Heading::ConstSharedPtr &heading, const mrover::msg::FixStatus::ConstSharedPtr &heading_status);
        void sync_imu_and_mag_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu, const mrover::msg::Heading::ConstSharedPtr &mag_heading);
        void drive_forward_callback();
        static auto quat_geodesic_angle_rad(const Eigen::Quaterniond &prev_quat, const Eigen::Quaterniond &current_quat) -> double;

        // subscribers and publishers
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr linearized_position_sub;
        message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub;
        message_filters::Subscriber<mrover::msg::Heading> mag_heading_sub;
        message_filters::Subscriber<mrover::msg::Heading> rtk_heading_sub;
        message_filters::Subscriber<mrover::msg::FixStatus> rtk_heading_status_sub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::ConstSharedPtr cmd_vel_sub;
        
        rclcpp::Publisher<mrover::msg::Heading>::SharedPtr drive_forward_pub;
        rclcpp::Publisher<mrover::msg::Heading>::SharedPtr imu_uncorrected_pub;

        // params
        std::string world_frame, gps_frame;
        double imu_timeout, mag_noise, rtk_noise, drive_noise, heading_delta_threshold, min_speed, process_noise;
        bool use_mag;

        // transform broadcaster
        tf2_ros::Buffer tf_buffer{get_clock()};
        tf2_ros::TransformListener tf_listener{tf_buffer};
        tf2_ros::TransformBroadcaster tf_broadcaster{this};

        // synchronizers
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime
            <mrover::msg::Heading, mrover::msg::FixStatus>>> rtk_heading_sync;
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime
            <sensor_msgs::msg::Imu, mrover::msg::Heading>>> imu_and_mag_sync;

        // imu data watchdog
        rclcpp::TimerBase::SharedPtr imu_and_mag_watchdog;
        rclcpp::TimerBase::SharedPtr drive_forward_timer;

        // 1D Kalman Filter state
        double X;
        double P;

        // data store
        std::optional<Eigen::Quaterniond> prev_imu_orientation_norm;
        std::optional<sensor_msgs::msg::Imu> last_imu;
        std::optional<geometry_msgs::msg::Vector3Stamped> last_position;
        std::vector<geometry_msgs::msg::Twist> twists;
        std::deque<geometry_msgs::msg::Vector3Stamped> position_window;    
        static constexpr std::size_t DRIVE_FORWARD_CAP = 3;
        static constexpr double DRIVE_FORWARD_TIMER_S = 1.25;
        static constexpr std::size_t TWISTS_CAP = 50;
        static constexpr std::size_t IMU_STUCK_THRESHOLD = 10;
        static constexpr std::size_t IMU_UNSTUCK_THRESHOLD = 10;
        static constexpr double EPS_STALE = 1e-9;
        std::size_t imu_stuck_counter = 0;
        std::size_t imu_unstuck_counter = 0;
        bool imu_orientation_stuck = false;


        //if no zed imu + zed imu stale = publish raw rtk yaw.
    
    public:

        HeadingFilter();

    }; // class HeadingFilter

} // namespace mrover