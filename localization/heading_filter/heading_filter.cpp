#include "heading_filter.hpp"

namespace mrover {

    HeadingFilter::HeadingFilter() : Node("heading_filter") {
        
        // publishers
        linearized_position_sub.subscribe(this, "/linearized_position", 1);
        imu_sub.subscribe(this, "/zed_imu/data_raw", 10);
        mag_sub.subscribe(this, "/zed_imu/mag", 10);
        rtk_heading_sub.subscribe(this, "/heading/fix", 1);
        rtk_heading_status_sub.subscribe(this, "/heading_fix_status", 1);

        // synchronizer
        uint32_t queue_size = 10;
        sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime
            <geometry_msgs::msg::Vector3Stamped, sensor_msgs::msg::Imu, sensor_msgs::msg::MagneticField, mrover::msg::Heading, mrover::msg::FixStatus>>>(
                message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::Vector3Stamped, sensor_msgs::msg::Imu, sensor_msgs::msg::MagneticField, mrover::msg::Heading, mrover::msg::FixStatus>(queue_size),
                linearized_position_sub,
                imu_sub,
                mag_sub,
                rtk_heading_sub,
                rtk_heading_status_sub
            );

        sync->setAgePenalty(0.50);
        sync->registerCallback(std::bind(&HeadingFilter::synced_position_orientation_callback, this, _1, _2));

    }

    void HeadingFilter::synced_position_orientation_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &position,
                                                            const sensor_msgs::msg::Imu::ConstSharedPtr &imu,
                                                            const sensor_msgs::msg::MagneticField::ConstSharedPtr &mag,
                                                            const mrover::msg::Heading::ConstSharedPtr &rtk_heading,
                                                            const mrover::msg::FixStatus::ConstSharedPtr &rtk_heading_status) {

        // to be implemented...
    
    }
} // namespace mrover