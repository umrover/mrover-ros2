#pragma once

#include "pch.hpp"

namespace mrover {

    class HeadingFilter : public rclcpp::Node {

    private:

        // callbacks
        void correct_and_publish(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &position);
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

        // data store
        std::optional<SO3d> curr_heading_correction;
        std::optional<sensor_msgs::msg::Imu> last_imu;
        std::optional<mrover::msg::Heading> last_mag_heading;

        // thresholding for data
        const rclcpp::Duration IMU_AND_MAG_WATCHDOG_TIMEOUT = rclcpp::Duration::from_seconds(1.0);
        constexpr static float HEADING_THRESHOLD = M_PI / 16;

        std::string rover_frame;
        std::string world_frame;

    
    public:

        HeadingFilter();

    }; // class HeadingFilter

} // namespace mrover