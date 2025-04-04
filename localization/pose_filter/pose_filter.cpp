#include "pose_filter.hpp"

namespace mrover {
    PoseFilter::PoseFilter() : Node("pose_filter") {

        // subscribers
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/zed_imu/data_raw", 1, const rclcpp::QoS &qos, CallbackT &&callback)
        pos_sub.subscribe(this, "/linearized_position");
        pos_status_sub.subscribe(this, "/heading_fix_status");
        
        //synchronise measurements 
        pos_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::Vector3Stamped, mrover::msg::FixStatus>>>(
            message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::Vector3Stamped, mrover::msg::FixStatus>(10),
            pos_sub,
            pos_status_sub
        );

        pos_sync->setAgePenalty(0.5);
        pos_sync->registerCallback(&PoseFilter::pose_callback, this);

        //transform

        imu_watchdog_timeout = this->create_wall_timer(imu_watchdog.to_chrono<std::chrono::milliseconds>(), [&]() {
            RCLCPP_WARN(get_logger(), "ZED IMU timed out!");
            last_imu.reset();
        })

        correction_timer = this->create_wall_timer() //we need a window for the correction step
    }

    void mrover::PoseFilter::pos_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &pos_msg, const mrover::msg::FixStatus::ConstSharedPtr& pos_status_msg) {
        if (pos_status_msg->fix_type.fix == mrover::msg::FixType::NO_SOL) {
            return;
        }

        last
    }

    void mrover::PoseFilter::pos_status_callback(const mrover::msg::FixStatus::ConstSharedPtr const& pos_status_msg) {
        
        
        if (pos_status_msg)
    }
    
}

