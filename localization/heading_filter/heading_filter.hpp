#pragma once

#include "pch.hpp"
#include <geometry_msgs/msg/detail/vector3_stamped__struct.hpp>
#include <sensor_msgs/msg/detail/magnetic_field__struct.hpp>

namespace mrover {

    using std::placeholders::_1;
    using std::placeholders::_2;

    class HeadingFilter : public rclcpp::Node {

    private:

        void correct_and_publish(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &position);

        // subscribers
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr linearized_position_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub;

        message_filters::Subscriber<mrover::msg::Heading> rtk_heading_sub;
        message_filters::Subscriber<mrover::msg::FixStatus> rtk_heading_status_sub;

        // rclcpp::Subscription<mrover::msg::Heading>::SharedPtr rtk_heading_sub;
        rclcpp::Subscription<mrover::msg::FixStatus>::SharedPtr rtk_heading_status_sub;

        // rtk heading data watchdog
        rclcpp::TimerBase::SharedPtr rtk_heading_watchdog;

        // data store
        std::optional<double> last_rtk_heading;
        std::optional<builtin_interfaces::msg::Time> last_rtk_heading_time;
        std::optional<SO3d> rtk_heading_correction_rotation;

        // thresholding for data
        constexpr static float HEADING_THRESHOLD = 10;
        constexpr static bool HEADING_CORRECTED = false;



        // void synced_position_orientation_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &position,
        //                                           const sensor_msgs::msg::Imu::ConstSharedPtr &imu,
        //                                           const sensor_msgs::msg::MagneticField::ConstSharedPtr &mag,
        //                                           const mrover::msg::Heading::ConstSharedPtr &rtk_heading,
        //                                           const mrover::msg::FixStatus::ConstSharedPtr &rtk_heading_status);
        
        // // subscribers
        // message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped> linearized_position_sub;
        // message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub;
        // message_filters::Subscriber<sensor_msgs::msg::MagneticField> mag_sub;
        // message_filters::Subscriber<mrover::msg::Heading> rtk_heading_sub;
        // message_filters::Subscriber<mrover::msg::FixStatus> rtk_heading_status_sub;

        // // synchronizer
        // std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime
        //     <geometry_msgs::msg::Vector3Stamped, sensor_msgs::msg::Imu, sensor_msgs::msg::MagneticField, mrover::msg::Heading, mrover::msg::FixStatus>>> sync;

    
    public:

        HeadingFilter();
        void spin();


    } // class HeadingFilter

} // namespace mrover