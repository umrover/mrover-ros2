#include "heading_filter.hpp"
#include <Eigen/src/Geometry/AngleAxis.h>
#include <rclcpp/logging.hpp>

namespace mrover {

    HeadingFilter::HeadingFilter() : Node("heading_filter") {

        // subscribers
        linearized_position_sub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("/linearized_position", 1, [&](const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &position) {
            correct_and_publish(position);
        });
        rtk_heading_sub.subscribe(this, "/heading/fix");
        rtk_heading_status_sub.subscribe(this, "/heading_fix_status");
        imu_sub.subscribe(this, "/zed_imu/data_raw");
        mag_heading_sub.subscribe(this, "/zed_imu/mag_heading");

        // imu data watchdog
        imu_and_mag_watchdog = this->create_wall_timer(IMU_AND_MAG_WATCHDOG_TIMEOUT.to_chrono<std::chrono::milliseconds>(), [&]() {
            RCLCPP_WARN(get_logger(), "ZED IMU data watchdog expired");
            last_imu.reset();
            last_mag_heading.reset();
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

        imu_and_mag_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, mrover::msg::Heading>>>(
            message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, mrover::msg::Heading>(queue_size),
            imu_sub,
            mag_heading_sub
        );

        imu_and_mag_sync->setAgePenalty(0.5);
        imu_and_mag_sync->registerCallback(&HeadingFilter::sync_imu_and_mag_callback, this);

        // tf tree parameters
        declare_parameter("rover_frame", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("world_frame", rclcpp::ParameterType::PARAMETER_STRING);

        rover_frame = get_parameter("rover_frame").as_string();
        world_frame = get_parameter("world_frame").as_string();
        

    }

    void HeadingFilter::sync_rtk_heading_callback(const mrover::msg::Heading::ConstSharedPtr &heading, const mrover::msg::FixStatus::ConstSharedPtr &heading_status) {

        if (!last_imu) {
            RCLCPP_WARN(get_logger(), "No IMU data!");
            return;
        }

        Eigen::Quaterniond uncorrected_orientation(last_imu->orientation.w, last_imu->orientation.x, last_imu->orientation.y, last_imu->orientation.z);
        R2d uncorrected_forward = uncorrected_orientation.toRotationMatrix().col(0).head(2);
        double uncorrected_heading = std::atan2(uncorrected_forward.y(), uncorrected_forward.x());

        // when a fixed rtk heading is received
        if (heading_status->fix_type.fix == mrover::msg::FixType::FIXED) {

            double measured_heading = fmod(heading->heading + 90, 360);
            if (measured_heading > 270) {
                measured_heading = measured_heading - 360;
            }
            measured_heading = (90 - measured_heading) * (M_PI / 180);

            double heading_correction_delta = measured_heading - uncorrected_heading;
            curr_heading_correction = Eigen::AngleAxisd(heading_correction_delta, R3d::UnitZ());

        }

        // magnetometer when correction already exists
        else if (curr_heading_correction) {
            
            double measured_heading = last_mag_heading->heading;
            if (measured_heading > 270) {
                measured_heading = measured_heading - 360;
            }
            measured_heading = (90 - measured_heading) * (M_PI / 180);

            double heading_correction_delta = measured_heading - uncorrected_heading;

            R2d prev_heading_correction_forward = curr_heading_correction->rotation().col(0).head(2);
            double prev_heading_correction_delta = std::atan2(prev_heading_correction_forward.y(), prev_heading_correction_forward.x());
            
            if (std::abs(prev_heading_correction_delta - heading_correction_delta) < HEADING_THRESHOLD) {
                curr_heading_correction = Eigen::AngleAxisd(heading_correction_delta, R3d::UnitZ());
            }

        }
        
        // magnetometer when correction does not exist
        else {

            double measured_heading = last_mag_heading->heading;
            if (measured_heading > 270) {
                measured_heading = measured_heading - 360;
            }
            measured_heading = (90 - measured_heading) * (M_PI / 180);

            double heading_correction_delta = measured_heading - uncorrected_heading;
            curr_heading_correction = Eigen::AngleAxisd(heading_correction_delta, R3d::UnitZ());

        }

        
    }

    void HeadingFilter::sync_imu_and_mag_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu, const mrover::msg::Heading::ConstSharedPtr &mag_heading) {
        imu_and_mag_watchdog.reset();
        last_imu = *imu;
        last_mag_heading = *mag_heading;
    }

    void HeadingFilter::correct_and_publish(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &position) {

        if (!last_imu) {
            RCLCPP_WARN(get_logger(), "No IMU data!");
            return;
        }

        R3d position_in_map(position->vector.x, position->vector.y, position->vector.z);
        SE3d pose_in_map(position_in_map, SO3d::Identity());

        Eigen::Quaterniond uncorrected_orientation(last_imu->orientation.w, last_imu->orientation.x, last_imu->orientation.y, last_imu->orientation.z);
        SO3d uncorrected_orientation_rotm = uncorrected_orientation;
        SO3d corrected_orientation = curr_heading_correction.value() * uncorrected_orientation_rotm;

        pose_in_map.asSO3() = corrected_orientation;
        
        SE3Conversions::pushToTfTree(tf_broadcaster, rover_frame, world_frame, pose_in_map, get_clock()->now());

        RCLCPP_INFO_STREAM(get_logger(), std::format("Heading corrected by: {}", curr_heading_correction->z()));
        
    }

}


int main(int argc, char**argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::HeadingFilter>());
    rclcpp::shutdown();
    return 0;

}