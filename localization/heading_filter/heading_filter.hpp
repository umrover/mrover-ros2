#pragma once

#include "mrover/msg/detail/fix_status__struct.hpp"
#include "pch.hpp"
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/detail/magnetic_field__struct.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace mrover {

    using std::placeholders::_1;
    using std::placeholders::_2;

    class HeadingFilter : public rclcpp::Node {

    private:

        void correct_and_publish(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &position);
        void sync_rtk_heading_callback(const mrover::msg::Heading::ConstSharedPtr &heading, const mrover::msg::FixStatus::ConstSharedPtr &heading_status);
        void sync_imu_and_mag_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu, const sensor_msgs::msg::MagneticField::ConstSharedPtr &mag);

        // subscribers and publishers
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr linearized_position_sub;
        // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        // rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub;

        message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub;
        message_filters::Subscriber<sensor_msgs::msg::MagneticField> mag_sub;
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
            <sensor_msgs::msg::Imu, sensor_msgs::msg::MagneticField>>> imu_and_mag_sync;

        // data watchdogs
        // rclcpp::TimerBase::SharedPtr rtk_heading_watchdog;
        rclcpp::TimerBase::SharedPtr imu_and_mag_watchdog;

        // data store
        // std::optional<double> last_rtk_heading;
        // std::optional<mrover::msg::FixType> last_rtk_heading_fix;
        // std::optional<builtin_interfaces::msg::Time> last_rtk_heading_time;
        // std::optional<SO3d> last_rtk_heading_correction_rotation;
        std::optional<SO3d> curr_heading_correction;
        std::optional<sensor_msgs::msg::Imu> last_imu;
        std::optional<sensor_msgs::msg::MagneticField> last_mag;

        // thresholding for data
        // const rclcpp::Duration RTK_HEADING_WATCHDOG_TIMEOUT = rclcpp::Duration::from_seconds(1.5);
        const rclcpp::Duration IMU_AND_MAG_WATCHDOG_TIMEOUT = rclcpp::Duration::from_seconds(1.0);
        constexpr static float HEADING_THRESHOLD = M_PI / 8;
        // constexpr static bool HEADING_CORRECTED = false;

        std::string rover_frame;
        std::string world_frame;

    
    public:

        HeadingFilter();

    }; // class HeadingFilter

} // namespace mrover