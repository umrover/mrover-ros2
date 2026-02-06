#include "heading_filter.hpp"

namespace mrover {

    HeadingFilter::HeadingFilter() : Node("heading_filter") {

        declare_parameter("world_frame", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("gps_frame", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("imu_watchdog_timeout", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("mag_heading_noise", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("rtk_heading_noise", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("process_noise", rclcpp::ParameterType::PARAMETER_DOUBLE);

        // subscribers
        linearized_position_sub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("/linearized_position", 1, [&](const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &position) {
            last_position = *position;
        });
        rtk_heading_sub.subscribe(this, "/heading/fix");
        rtk_heading_status_sub.subscribe(this, "/heading_fix_status");
        imu_sub.subscribe(this, "/zed_imu/data_raw");
        mag_heading_sub.subscribe(this, "/zed_imu/mag_heading");

        // imu data watchdog
        const rclcpp::Duration IMU_AND_MAG_WATCHDOG_TIMEOUT = rclcpp::Duration::from_seconds(get_parameter("imu_watchdog_timeout").as_double());
        imu_and_mag_watchdog = this->create_wall_timer(IMU_AND_MAG_WATCHDOG_TIMEOUT.to_chrono<std::chrono::milliseconds>(), [&]() {
            RCLCPP_WARN(get_logger(), "ZED IMU data watchdog expired");
            last_imu.reset();
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

        X = 0;
        P = 1;

    }

    void HeadingFilter::predict(double process_noise) {

        P = P + process_noise;

    }


    void HeadingFilter::correct(double heading_correction_delta_meas, double heading_correction_delta_noise) {

        double K = (P) / (P + heading_correction_delta_noise);
        double innovation = heading_correction_delta_meas - X;
        innovation = fmod((innovation + 3 * M_PI), 2 * M_PI) - M_PI;
        X = fmod((X + K * (innovation) + 3 * M_PI), 2 * M_PI) - M_PI;
        P = (1 - K) * P;

    }

    void HeadingFilter::sync_rtk_heading_callback(const mrover::msg::Heading::ConstSharedPtr &heading, const mrover::msg::FixStatus::ConstSharedPtr &heading_status) {


        if (!last_imu) {
            RCLCPP_WARN(get_logger(), "No IMU data!");
            return;
        }

        auto const& qmsg2 = last_imu->orientation;
        if (!std::isfinite(qmsg2.w) || !std::isfinite(qmsg2.x) || !std::isfinite(qmsg2.y) || !std::isfinite(qmsg2.z)) {
            RCLCPP_WARN(get_logger(), "IMU quaternion as a non-finite component, skipping orientation");
            return;
        }
        Eigen::Quaterniond uncorrected_orientation(qmsg2.w, qmsg2.x, qmsg2.y, qmsg2.z);
        double const norm2 = uncorrected_orientation.squaredNorm();
        if (!std::isfinite(norm2) || norm2 < 1e-12) {
            RCLCPP_WARN(get_logger(), "IMU quaternion has an invalid/near-zero norm, skipping normalization");
            return;
        }
        uncorrected_orientation.normalize();
        R2d uncorrected_forward = uncorrected_orientation.toRotationMatrix().col(0).head(2);
        if (!uncorrected_forward.array().isFinite().all()) {
            RCLCPP_WARN(get_logger(), "Forward vector not finite, skipping heading correction");
            return;
        }
        double uncorrected_heading = std::atan2(uncorrected_forward.y(), uncorrected_forward.x());
        if (!std::isfinite(uncorrected_heading)) {
            RCLCPP_WARN(get_logger(), "Computed heading not finite, skipping heading correction");
            return;
        }
        // correct with rtk heading only when heading is fixed
        double measured_heading = fmod(heading->heading + 90, 360);
        measured_heading = 90 - measured_heading;
        if (measured_heading < -180) {
            measured_heading = measured_heading + 360;
        }
        measured_heading = measured_heading * (M_PI / 180);

        double heading_correction_delta = measured_heading - uncorrected_heading;
        heading_correction_delta = fmod((heading_correction_delta + 3 * M_PI), 2 * M_PI) - M_PI;
        predict(get_parameter("process_noise").as_double());
        correct(heading_correction_delta, get_parameter("rtk_heading_noise").as_double());
    }     

    void HeadingFilter::sync_imu_and_mag_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu, const mrover::msg::Heading::ConstSharedPtr &mag_heading) {
        imu_and_mag_watchdog.reset();

        last_imu = *imu;

        if (!last_position) {
            RCLCPP_WARN(get_logger(), "No position data!");
            return;
        }

        R3d position_in_map(last_position->vector.x, last_position->vector.y, last_position->vector.z);
        SE3d pose_in_map(position_in_map, SO3d::Identity());

        auto const& qmsg = imu->orientation;
        if (!std::isfinite(qmsg.w) || !std::isfinite(qmsg.x) || !std::isfinite(qmsg.y) || !std::isfinite(qmsg.z)) {
            RCLCPP_WARN(get_logger(), "IMU quaternion has a non-finite component, skipping normalization");
            return;
        }
        // correct with mag heading
        Eigen::Quaterniond uncorrected_orientation(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
        double const norm2 = uncorrected_orientation.squaredNorm();
        if (!std::isfinite(norm2) || norm2 < 1e-12) {
            RCLCPP_WARN(get_logger(), "IMU quaternion has invalid/near-zero norm; skipping orientation update");
            return;
        }
        uncorrected_orientation.normalize();
        SO3d uncorrected_orientation_rotm = uncorrected_orientation;
        R2d uncorrected_forward = uncorrected_orientation.toRotationMatrix().col(0).head(2);
        if (!uncorrected_forward.array().isFinite().all()) {
            RCLCPP_WARN(get_logger(), "Forward Vector not finite, skipping heading correction");
            return;
        }
        double uncorrected_heading = std::atan2(uncorrected_forward.y(), uncorrected_forward.x());
        if (!std::isfinite(uncorrected_heading)) {
            RCLCPP_WARN(get_logger(), "Computed heading is not finite, skipping heading correction");
            return;
        }

        double measured_heading = 90 - mag_heading->heading;
        if (measured_heading < -180) {
            measured_heading = 360 + measured_heading;
        }
        measured_heading = measured_heading * (M_PI / 180);

        double heading_correction_delta = measured_heading - uncorrected_heading;
        heading_correction_delta = fmod((heading_correction_delta + 3 * M_PI), 2 * M_PI) - M_PI;
        predict(get_parameter("process_noise").as_double());
        correct(heading_correction_delta, get_parameter("mag_heading_noise").as_double());

        // apply correction and publish
        SO3d curr_heading_correction = Eigen::AngleAxisd(X, R3d::UnitZ());
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", std::format("Heading corrected by: {} rad", X).c_str());
        SO3d corrected_orientation = curr_heading_correction * uncorrected_orientation_rotm;
        pose_in_map.asSO3() = corrected_orientation;

        SE3Conversions::pushToTfTree(tf_broadcaster, get_parameter("gps_frame").as_string(), get_parameter("world_frame").as_string(), pose_in_map, get_clock()->now());
    }
}


int main(int argc, char**argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::HeadingFilter>());
    rclcpp::shutdown();
    return 0;

}