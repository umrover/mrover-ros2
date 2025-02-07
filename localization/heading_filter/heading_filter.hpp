#pragma once

#include "mrover/msg/detail/fix_status__struct.hpp"
#include "pch.hpp"

namespace mrover {

    using std::placeholders::_1;
    using std::placeholders::_2;

    class HeadingFilter : public rclcpp::Node {

    private:

        void correct_and_publish(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &position);
        void sync_rtk_heading_callback(const mrover::msg::Heading::ConstSharedPtr &heading, const mrover::msg::FixStatus::ConstSharedPtr &heading_status);

        // subscribers
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr linearized_position_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub;

        message_filters::Subscriber<mrover::msg::Heading> rtk_heading_sub;
        message_filters::Subscriber<mrover::msg::FixStatus> rtk_heading_status_sub;

        // synchronizer
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime
            <mrover::msg::Heading, mrover::msg::FixStatus>>> sync;

        // rtk heading data watchdog
        rclcpp::TimerBase::SharedPtr rtk_heading_watchdog;

        // data store
        std::optional<double> last_rtk_heading;
        std::optional<mrover::msg::FixType> last_rtk_heading_fix;
        std::optional<builtin_interfaces::msg::Time> last_rtk_heading_time;
        // std::optional<SO3d> last_rtk_heading_correction_rotation;
        std::optional<SO3d> curr_heading_correction;
        std::optional<sensor_msgs::msg::Imu> last_imu;
        std::optional<sensor_msgs::msg::MagneticField> last_mag;

        // thresholding for data
        const rclcpp::Duration RTK_HEADING_WATCHDOG_TIMEOUT = rclcpp::Duration::from_seconds(2.0);
        constexpr static float HEADING_THRESHOLD = 10;
        // constexpr static bool HEADING_CORRECTED = false;

       

        

        // rclcpp::Subscription<mrover::msg::Heading>::SharedPtr rtk_heading_sub;
        // rclcpp::Subscription<mrover::msg::FixStatus>::SharedPtr rtk_heading_status_sub;



        // void synced_position_orientation_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &position,
        //                                           const sensor_msgs::msg::Imu::ConstSharedPtr &imu,
        //                                           const sensor_msgs::msg::MagneticField::ConstSharedPtr &mag,
        //                                           const mrover::msg::Heading::ConstSharedPtr &rtk_heading,
        //                                           const mrover::msg::FixStatus::ConstSharedPtr &rtk_heading_status);
        
        // // subscribers
        // message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped> linearized_position_sub;
        // message_filters::Subscribusing std::placeholders::_1;
    // using std::placeholders::_2;er<sensor_msgs::msg::Imu> imu_sub;
        // message_filters::Subscriber<sensor_msgs::msg::MagneticField> mag_sub;
        // message_filters::Subscriber<mrover::msg::Heading> rtk_heading_sub;
        // message_filters::Subscriber<mrover::msg::FixStatus> rtk_heading_status_sub;

        // // synchronizer
        // std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime
        //     <geometry_msgs::msg::Vector3Stamped, sensor_msgs::msg::Imu, sensor_msgs::msg::MagneticField, mrover::msg::Heading, mrover::msg::FixStatus>>> sync;

    
    public:

        HeadingFilter();

    }; // class HeadingFilter

} // namespace mrover