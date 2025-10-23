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

        // subscribers and publishers
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr linearized_position_sub;
        message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub;
        message_filters::Subscriber<mrover::msg::Heading> mag_heading_sub;
        message_filters::Subscriber<mrover::msg::Heading> rtk_heading_sub;
        message_filters::Subscriber<mrover::msg::FixStatus> rtk_heading_status_sub;

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

        // 1D Kalman Filter state
        double X;
        double P;

        // data store
        std::optional<sensor_msgs::msg::Imu> last_imu;
        std::optional<geometry_msgs::msg::Vector3Stamped> last_position;
    
    public:

        HeadingFilter();

    }; // class HeadingFilter

} // namespace mrover