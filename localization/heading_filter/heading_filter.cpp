#include "heading_filter.hpp"
#include "mrover/msg/detail/fix_status__struct.hpp"
#include <chrono>
#include <cstdint>
#include <geometry_msgs/msg/detail/vector3_stamped__struct.hpp>
#include <memory>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/detail/magnetic_field__struct.hpp>

namespace mrover {

    HeadingFilter::HeadingFilter() : Node("heading_filter") {

        // subscribers
        linearized_position_sub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("/linearized_position", 1, [&](const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &position) {
            correct_and_publish(position);
        });

        // imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/zed_imu/data_raw", 10, [&](const sensor_msgs::msg::Imu::ConstSharedPtr &imu) {
        //     last_imu = *imu;
        // });

        // mag_sub = this->create_subscription<sensor_msgs::msg::MagneticField>("/zed_imu/mag", 10, [&](const sensor_msgs::msg::MagneticField::ConstSharedPtr &mag) {
        //     last_mag = *mag;
        // });

        rtk_heading_sub.subscribe(this, "/heading/fix");
        rtk_heading_status_sub.subscribe(this, "/heading_fix_status");
        imu_sub.subscribe(this, "/zed_imu/data_raw");
        mag_sub.subscribe(this, "/zed_imu/mag");

        // data watchdogs
        rtk_heading_watchdog = this->create_wall_timer(RTK_HEADING_WATCHDOG_TIMEOUT.to_chrono<std::chrono::milliseconds>(), [&]() {
            RCLCPP_WARN(get_logger(), "RTK heading data watchdog expired");
            last_rtk_heading.reset();
            last_rtk_heading_fix.reset();
            // last_rtk_heading_time.reset();

            // last_rtk_heading_fix.reset();
            // last_rtk_heading_correction_rotation.reset();
        });

        imu_and_mag_watchdog = this->create_wall_timer(IMU_AND_MAG_WATCHDOG_TIMEOUT.to_chrono<std::chrono::milliseconds>(), [&]() {
            RCLCPP_WARN(get_logger(), "ZED IMU data watchdog expired");
            last_imu.reset();
            last_mag.reset();
        });

        // synchronizers
        uint32_t queue_size = 10;

        rtk_heading_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<mrover::msg::Heading, mrover::msg::FixStatus>>>(
            message_filters::sync_policies::ApproximateTime<mrover::msg::Heading, mrover::msg::FixStatus>(queue_size),
            rtk_heading_sub,
            rtk_heading_status_sub
        );

        rtk_heading_sync->setAgePenalty(0.5);
        rtk_heading_sync->registerCallback(&HeadingFilter::sync_rtk_heading_callback, this);

        imu_and_mag_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::MagneticField>>>(
            message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::MagneticField>(queue_size),
            imu_sub,
            mag_sub
        );

        imu_and_mag_sync->setAgePenalty(0.5);
        imu_and_mag_sync->registerCallback(&HeadingFilter::sync_imu_and_mag_callback, this);
        

    }

    void HeadingFilter::sync_rtk_heading_callback(const mrover::msg::Heading::ConstSharedPtr &heading, const mrover::msg::FixStatus::ConstSharedPtr &heading_status) {
        rtk_heading_watchdog.reset();
        last_rtk_heading = heading->heading;
        last_rtk_heading_fix = heading_status->fix_type;
        // last_rtk_heading_time = heading->header.stamp;
    }

    // void HeadingFilter::sync_imu_and_mag_callback(const sensor_msgs::msg::)

    void HeadingFilter::correct_and_publish(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &position) {

        if (!last_imu) {
            RCLCPP_WARN(get_logger(), "No IMU data!");
            return;
        }
        
        if (!curr_heading_correction) {

            if (last_rtk_heading_fix->fix == mrover::msg::FixType::FIXED) {
                

            }
        }
    }

}

// namespace mrover {

//     HeadingFilter::HeadingFilter() : Node("heading_filter") {
        
//         // publishers
//         linearized_position_sub.subscribe(this, "/linearized_position", 1);
//         imu_sub.subscribe(this, "/zed_imu/data_raw", 10);
//         mag_sub.subscribe(this, "/zed_imu/mag", 10);
//         rtk_heading_sub.subscribe(this, "/heading/fix", 1);
//         rtk_heading_status_sub.subscribe(this, "/heading_fix_status", 1);

//         // synchronizer
//         uint32_t queue_size = 10;
//         sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime
//             <geometry_msgs::msg::Vector3Stamped, sensor_msgs::msg::Imu, sensor_msgs::msg::MagneticField, mrover::msg::Heading, mrover::msg::FixStatus>>>(
//                 message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::Vector3Stamped, sensor_msgs::msg::Imu, sensor_msgs::msg::MagneticField, mrover::msg::Heading, mrover::msg::FixStatus>(queue_size),
//                 linearized_position_sub,
//                 imu_sub,
//                 mag_sub,
//                 rtk_heading_sub,
//                 rtk_heading_status_sub
//             );

//         sync->setAgePenalty(0.50);
//         sync->registerCallback(std::bind(&HeadingFilter::synced_position_orientation_callback, this, _1, _2));

//     }

//     void HeadingFilter::synced_position_orientation_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &position,
//                                                             const sensor_msgs::msg::Imu::ConstSharedPtr &imu,
//                                                             const sensor_msgs::msg::MagneticField::ConstSharedPtr &mag,
//                                                             const mrover::msg::Heading::ConstSharedPtr &rtk_heading,
//                                                             const mrover::msg::FixStatus::ConstSharedPtr &rtk_heading_status) {

//         // to be implemented...
    
//     }
// } // namespace mrover



int main(int argc, char**argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::HeadingFilter>());
    rclcpp::shutdown();
    return 0;

}