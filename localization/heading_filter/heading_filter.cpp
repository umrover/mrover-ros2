#include "heading_filter.hpp"
#include <Eigen/src/Geometry/AngleAxis.h>
#include <rclcpp/logging.hpp>

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
        // mag_sub.subscribe(this, "/zed_imu/mag");
        mag_heading_sub.subscribe(this, "/zed_imu/mag_heading");

        // data watchdogs
        // rtk_heading_watchdog = this->create_wall_timer(RTK_HEADING_WATCHDOG_TIMEOUT.to_chrono<std::chrono::milliseconds>(), [&]() {
        //     RCLCPP_WARN(get_logger(), "RTK heading data watchdog expired");
        //     last_rtk_heading.reset();
        //     last_rtk_heading_fix.reset();
        //     // last_rtk_heading_time.reset();

        //     // last_rtk_heading_fix.reset();
        //     // last_rtk_heading_correction_rotation.reset();
        // });

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

        declare_parameter("rover_frame", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("world_frame", rclcpp::ParameterType::PARAMETER_STRING);

        rover_frame = get_parameter("rover_frame").as_string();
        world_frame = get_parameter("world_frame").as_string();
        

    }

    void HeadingFilter::sync_rtk_heading_callback(const mrover::msg::Heading::ConstSharedPtr &heading, const mrover::msg::FixStatus::ConstSharedPtr &heading_status) {
        // rtk_heading_watchdog.reset();
        // last_rtk_heading = heading->heading;
        // last_rtk_heading_fix = heading_status->fix_type;
        // last_rtk_heading_time = heading->header.stamp;

        if (!last_imu) {
            RCLCPP_WARN(get_logger(), "No IMU data!");
            return;
        }


        Eigen::Quaterniond uncorrected_orientation(last_imu->orientation.w, last_imu->orientation.x, last_imu->orientation.y, last_imu->orientation.z);
        R2d uncorrected_forward = uncorrected_orientation.toRotationMatrix().col(0).head(2);
        double uncorrected_heading = std::atan2(uncorrected_forward.y(), uncorrected_forward.x());

        // when a fixed rtk heading is received
        if (heading_status->fix_type.fix == mrover::msg::FixType::FIXED) {

            double measured_heading = (90 - (fmod(heading->heading + 90, 360))) * (M_PI / 180.0);
            double heading_correction_delta = measured_heading - uncorrected_heading;
            curr_heading_correction = Eigen::AngleAxisd(heading_correction_delta, R3d::UnitZ());

        }
        // magnetometer when correction already exists
        else if (curr_heading_correction) {
            // double measured_heading = (M_PI / 2) - std::atan2(last_mag->magnetic_field.y, last_mag->magnetic_field.x);
            double measured_heading = last_mag_heading->heading;

            if (measured_heading > 270) {
                measured_heading = measured_heading - 360;
            }

            measured_heading = (90 - measured_heading) * (M_PI / 180);

            double heading_correction_delta = measured_heading - uncorrected_heading;

            R2d prev_heading_correction_forward = curr_heading_correction->rotation().col(0).head(2);

            // R2d prev_heading_correction_forward = curr_heading_correction.value().coeffs().col(0).head(2);

            // Eigen::MatrixXd mat = curr_heading_correction.value().coeffs();

            // RCLCPP_INFO_STREAM(get_logger(), std::format("curr_heading_correction: {} {} {}", mat(0,0), mat(0, 1), mat(0, 2)));
            // RCLCPP_INFO_STREAM(get_logger(), std::format("curr_heading_correction: {} {} {}", mat(1,0), mat(1, 1), mat(1, 2)));
            // RCLCPP_INFO_STREAM(get_logger(), std::format("curr_heading_correction: {} {} {}", mat(2,0), mat(2, 1), mat(2, 2)));
            // RCLCPP_INFO_STREAM(get_logger(), std::format("curr_heading_correction (0,0)", mat(0,0)));
            double prev_heading_correction_delta = std::atan2(prev_heading_correction_forward.y(), prev_heading_correction_forward.x());
            
            // compare curr_heading_correction with correction made from mag
            // RCLCPP_INFO_STREAM(get_logger(), std::format("heading difference: {}", std::abs(prev_heading_correction_delta - heading_correction_delta)));
            // RCLCPP_INFO_STREAM(get_logger(), std::format("prev heading correction delta: {}", prev_heading_correction_delta));
            // RCLCPP_INFO_STREAM(get_logger(), std::format("heading correction delta: {}", heading_correction_delta));
            // double measured_heading = (M_PI / 2) - std::atan2(last_mag->magnetic_field.y, last_mag->magnetic_field.x);/ RCLCPP_INFO_STREAM(get_logger(), std::format("mag heading: {}", measured_heading));
            RCLCPP_INFO_STREAM(get_logger(), std::format("mag heading: {}", measured_heading));
            RCLCPP_INFO_STREAM(get_logger(), std::format("uncorrected heading: {}", uncorrected_heading));
            
            
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

            Eigen::MatrixXd mat = curr_heading_correction->rotation();
            // curr_heading_correction = correction.toRotationMatrix();


            // RCLCPP_INFO_STREAM(get_logger(), std::format("measured heading: {}", measured_heading));
            // RCLCPP_INFO_STREAM(get_logger(), std::format("uncorrected heading: {}", uncorrected_heading));
            // RCLCPP_INFO_STREAM(get_logger(), std::format("heading correction delta: {}", heading_correction_delta));
            // curr_heading_correction = Eigen::AngleAxisd(heading_correction_delta, R3d::UnitZ());

            // Eigen::AngleAxisd rotation_temp = Eigen::AngleAxisd(heading_correction_delta, Eigen::Vector3d::UnitZ());
            // Eigen::Matrix3d mat = rotation_temp.toRotationMatrix();

            // Eigen::Matrix3d mat = curr_heading_correction.value().coeffs();

            // Eigen::Matrix3d mat = curr_heading_correction.value().coeffs();

            RCLCPP_INFO_STREAM(get_logger(), std::format("curr_heading_correction: {} {} {}", mat(0,0), mat(0, 1), mat(0, 2)));
            RCLCPP_INFO_STREAM(get_logger(), std::format("curr_heading_correction: {} {} {}", mat(1,0), mat(1, 1), mat(1, 2)));
            RCLCPP_INFO_STREAM(get_logger(), std::format("curr_heading_correction: {} {} {}", mat(2,0), mat(2, 1), mat(2, 2)));
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