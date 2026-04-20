#include "heading_filter.hpp"

namespace mrover {

    auto HeadingFilter::quat_geodesic_angle_rad(const Eigen::Quaterniond &prev_quat, const Eigen::Quaterniond &current_quat) -> double {
        Eigen::Quaterniond a_n = prev_quat.normalized();
        Eigen::Quaterniond b_n = current_quat.normalized();

        Eigen::Quaterniond q_rel = b_n * a_n.conjugate();
        if (q_rel.w() < 0) {
            q_rel.coeffs() *= -1.0;
        }
        q_rel.normalize();
        double const half_angle = std::atan2(q_rel.vec().norm(), std::abs(q_rel.w()));
        return 2.0 * half_angle;
    }

    HeadingFilter::HeadingFilter() : Node("heading_filter") {

        std::vector<ParameterWrapper> params{
            {"world_frame", world_frame, std::string("map")},
            {"gps_frame", gps_frame, std::string("gps_frame")},
            {"imu_watchdog_timeout", imu_timeout, 1.0},
            {"mag_heading_noise", mag_noise, 10.0},
            {"rtk_heading_noise", rtk_noise, 0.00001},
            {"drive_forward_heading_noise", drive_noise, 0.0001},
            {"rover_heading_change_threshold", heading_delta_threshold, 0.05},
            {"minimum_linear_speed", min_speed, 0.4},
            {"process_noise", process_noise, 0.000001},
            {"use_mag", use_mag, false},
            {"use_rtk_only", use_rtk_only, false},
        };

        ParameterWrapper::declareParameters(this, params);

        // subscribers
        linearized_position_sub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("/linearized_position", 1, [&](const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &position) {
            last_position = *position;
            position_window.push_back(*position);
            while (position_window.size() > DRIVE_FORWARD_CAP) {
                position_window.pop_front();
            }
        });

        rtk_heading_sub.subscribe(this, "/heading/fix");
        rtk_heading_status_sub.subscribe(this, "/heading_fix_status");

        // synchronizers
        uint32_t queue_size = 10;

        rtk_heading_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<mrover::msg::Heading, mrover::msg::FixStatus>>>(
            message_filters::sync_policies::ApproximateTime<mrover::msg::Heading, mrover::msg::FixStatus>(queue_size),
            rtk_heading_sub,
            rtk_heading_status_sub
        );

        rtk_heading_sync->setAgePenalty(0.5);
        rtk_heading_sync->registerCallback(&HeadingFilter::sync_rtk_heading_callback, this);

        if (!use_rtk_only) {
            imu_sub.subscribe(this, "/zed_imu/data_raw");
            mag_heading_sub.subscribe(this, "/zed_imu/mag_heading");

            cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, [&](const geometry_msgs::msg::Twist::ConstSharedPtr &twist_msg) {
                twists.push_back(*twist_msg);
                if (twists.size() > TWISTS_CAP) {
                    twists.erase(twists.begin(), twists.begin() + (twists.size() - TWISTS_CAP));
                }
            });

            drive_forward_pub = this->create_publisher<mrover::msg::Heading>("/drive_forward_heading", 1);
            imu_uncorrected_pub = this->create_publisher<mrover::msg::Heading>("/imu_uncorrected_heading", 1);

            // imu data watchdog
            const rclcpp::Duration IMU_AND_MAG_WATCHDOG_TIMEOUT = rclcpp::Duration::from_seconds(imu_timeout);
            imu_and_mag_watchdog = this->create_wall_timer(IMU_AND_MAG_WATCHDOG_TIMEOUT.to_chrono<std::chrono::milliseconds>(), [&]() {
                RCLCPP_WARN(get_logger(), "ZED IMU data watchdog expired");
                last_imu.reset();
                prev_imu_orientation_norm.reset();
                imu_stuck_counter = 0;
                imu_unstuck_counter = 0;
                imu_orientation_stuck = false;
            });

            // drive forward correction timer
            drive_forward_timer = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int64_t>(DRIVE_FORWARD_TIMER_S * 1000.0)),
                [this]() -> void { drive_forward_callback(); }
            );

            imu_and_mag_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, mrover::msg::Heading>>>(
                message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, mrover::msg::Heading>(queue_size),
                imu_sub,
                mag_heading_sub
            );

            imu_and_mag_sync->setAgePenalty(0.5);
            imu_and_mag_sync->registerCallback(&HeadingFilter::sync_imu_and_mag_callback, this);
        } else {
            RCLCPP_WARN(get_logger(), "Heading filter publishing in rtk-yaw-only mode");
        }

        X = 0;
        P = 1;

    }

    void HeadingFilter::predict(double process_noise) {

        P = P + process_noise;

    }


    void HeadingFilter::correct(double heading_correction_delta_meas, double heading_correction_delta_noise) {

        double K = (P) / (P + heading_correction_delta_noise);
        double innovation = heading_correction_delta_meas - X;
        X = std::fmod((X + K * (innovation) + 3 * M_PI), 2 * M_PI) - M_PI;
        P = (1 - K) * P;
    }

    void HeadingFilter::sync_rtk_heading_callback(const mrover::msg::Heading::ConstSharedPtr &heading, const mrover::msg::FixStatus::ConstSharedPtr &heading_status) {

        if (heading_status->fix_type.fix == mrover::msg::FixType::FIXED) {
            double const rover_map_deg = fmod(heading->heading + 90. + 360., 360.);
            double measured_heading_deg = 90. - rover_map_deg;
            if (measured_heading_deg <= -180.) { measured_heading_deg += 360.; }
            else if (measured_heading_deg > 180.) { measured_heading_deg -= 360.; }
            if (use_rtk_only) {
                if (!last_position) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "RTK-yaw only mode, waiting for /linearized_pos");
                    return;
                }
                R3d position_in_map(last_position->vector.x, last_position->vector.y, last_position->vector.z);
                double const measured_heading = measured_heading_deg * (M_PI / 180.);
                SE3d pose_in_map(position_in_map, SO3d(Eigen::AngleAxisd(measured_heading, R3d::UnitZ())));
                SE3Conversions::pushToTfTree(tf_broadcaster, gps_frame, world_frame, pose_in_map, get_clock()->now());
                return;
            }
        }
        
        if (use_rtk_only) {
            return;
        }

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

        if (heading_status->fix_type.fix == mrover::msg::FixType::FIXED) {
            double const rover_map_deg = fmod(heading->heading + 90. + 360., 360.);
            double measured_heading_deg = 90. - rover_map_deg;
            if (measured_heading_deg <= -180.) { measured_heading_deg += 360.; }
            else if (measured_heading_deg > 180.) { measured_heading_deg -= 360.; }
            double const measured_heading = measured_heading_deg * (M_PI / 180.);

            auto const previousX = X;

            double heading_correction_delta = measured_heading - uncorrected_heading;
            heading_correction_delta = std::fmod((heading_correction_delta + 3 * M_PI), 2 * M_PI) - M_PI;
            predict(process_noise);
            correct(heading_correction_delta, rtk_noise);

            auto const correctionDelta = (X - previousX);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", std::format("RTK heading correction delta on X: {} rad", correctionDelta).c_str());
        }
    }

    void HeadingFilter::sync_imu_and_mag_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu, const mrover::msg::Heading::ConstSharedPtr &mag_heading) {
        if (use_rtk_only) {
            return;
        }
        
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
        Eigen::Quaterniond uncorrected_orientation(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
        double const norm2 = uncorrected_orientation.squaredNorm();
        if (!std::isfinite(norm2) || norm2 < 1e-12) {
            RCLCPP_WARN(get_logger(), "IMU quaternion has invalid/near-zero norm; skipping orientation update");
            return;
        }
        uncorrected_orientation.normalize();
        SO3d uncorrected_orientation_rotm = uncorrected_orientation;

        // For debugging purposes, calculating raw imu heading outside checks
        R2d uncorrected_forward = uncorrected_orientation.toRotationMatrix().col(0).head(2);
        if (uncorrected_forward.array().isFinite().all()) {
            double uncorrected_heading = std::atan2(uncorrected_forward.y(), uncorrected_forward.x());
            if (std::isfinite(uncorrected_heading)) {
                mrover::msg::Heading imu_heading_msg;
                imu_heading_msg.heading = std::fmod(90. - (uncorrected_heading * (180. / M_PI)) + 360., 360.);
                imu_uncorrected_pub->publish(imu_heading_msg);
            }
        }

        if (prev_imu_orientation_norm) {
            double const theta = quat_geodesic_angle_rad(*prev_imu_orientation_norm, uncorrected_orientation);
            if (theta < EPS_STALE) {
                imu_stuck_counter++;
                imu_unstuck_counter = 0;
            } else if (theta >= EPS_STALE) {
                imu_unstuck_counter++;
                imu_stuck_counter = 0;
            }
        } else {
            imu_stuck_counter = 0;
            imu_unstuck_counter = 0;
        }

        if (imu_orientation_stuck) {
            if (imu_unstuck_counter >= IMU_UNSTUCK_THRESHOLD) {
                imu_orientation_stuck = false;
                imu_unstuck_counter = 0;
                RCLCPP_WARN(get_logger(), "IMU orientation no longer stuck, resuming IMU-based corrections");
            }
        } else {
            if (imu_stuck_counter >= IMU_STUCK_THRESHOLD) {
                imu_orientation_stuck = true;
                imu_stuck_counter = 0;
                RCLCPP_WARN(get_logger(), "IMU orientation appears stuck (constant quaternion)");
            }
        }

        prev_imu_orientation_norm = uncorrected_orientation;

        if (use_mag) {
            R2d uncorrected_forward = uncorrected_orientation_rotm.rotation().col(0).head(2);
            if (!uncorrected_forward.array().isFinite().all()) {
                RCLCPP_WARN(get_logger(), "Forward Vector not finite, skipping heading correction");
                return;
            }
            double uncorrected_heading = std::atan2(uncorrected_forward.y(), uncorrected_forward.x());
            if (!std::isfinite(uncorrected_heading)) {
                RCLCPP_WARN(get_logger(), "Computed heading is not finite, skipping heading correction");
                return;
            }

            double measured_heading_deg = 90. - mag_heading->heading;
            if (measured_heading_deg <= -180.) { measured_heading_deg += 360.; }
            else if (measured_heading_deg > 180.) { measured_heading_deg -= 360.; }
            double const measured_heading = measured_heading_deg * (M_PI / 180.);

            double heading_correction_delta = measured_heading - uncorrected_heading;
            heading_correction_delta = std::fmod((heading_correction_delta + 3 * M_PI), 2 * M_PI) - M_PI;

            auto const previousX = X;

            predict(process_noise);
            correct(heading_correction_delta, mag_noise);

            if (!std::isfinite(X)) {
                RCLCPP_WARN(get_logger(), "Kalman state X is not finite, skipping TF publish");
                return;
            }

            auto const correctionDelta = (X - previousX);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", std::format("Mag heading correction delta on X: {} rad", correctionDelta).c_str());
        }

        if (!std::isfinite(X)) {
            RCLCPP_WARN(get_logger(), "Kalman state X is not finite, skipping TF publish");
            return;
        }

        SO3d curr_heading_correction = Eigen::AngleAxisd(X, R3d::UnitZ());
        SO3d corrected_orientation = curr_heading_correction * uncorrected_orientation_rotm;
        Eigen::Quaterniond q = corrected_orientation.quat();
        q.normalize();
        if (!q.coeffs().array().isFinite().all() || q.squaredNorm() < 1e-12) {
            RCLCPP_WARN(get_logger(), "Corrected orientation quaternion invalid after normalize, skipping TF publish");
            return;
        }
        pose_in_map.asSO3() = SO3d(q);

        SE3Conversions::pushToTfTree(tf_broadcaster, gps_frame, world_frame, pose_in_map, get_clock()->now());
    }

    void HeadingFilter::drive_forward_callback() {
        if (use_rtk_only) {
            return;
        }

        if (!last_imu) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No IMU data for drive-forward correction");
            return;
        }

        double const min_linear_speed = min_speed;

        // Gate on "commanded forward"
        bool const had_cmd_vel_samples = !twists.empty();
        double mean_cmd_vel = 0.0;
        if (had_cmd_vel_samples) {
            for (auto const& twist : twists) {
                mean_cmd_vel += twist.linear.x / static_cast<double>(twists.size());
            }
        }
        twists.clear();
        if (mean_cmd_vel < min_linear_speed) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Drive-forward skipped: mean cmd_vel.linear.x %.3f m/s below minimum_linear_speed %.3f m/s%s",
                mean_cmd_vel,
                min_linear_speed,
                had_cmd_vel_samples ? "" : " (no cmd_vel samples in buffer)");
            return;
        }

        if (position_window.size() < 2) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Insufficient position history for drive-forward correction");
            return;
        }

        // Estimate mean velocity and ensure near-straight motion over the window.
        R2d v_sum = R2d::Zero();
        double heading_change_accum = 0.0;
        std::size_t readings = 0;

        auto wrapped_delta = [](double a) -> double {
            return std::fmod(a + 3 * M_PI, 2 * M_PI) - M_PI;
        };

        auto prev_heading_opt = std::optional<double>{};
        rcl_clock_type_t const clock_type = get_clock()->get_clock_type();
        for (std::size_t i = 1; i < position_window.size(); ++i) {
            const auto& p0 = position_window[i - 1];
            const auto& p1 = position_window[i];

            const rclcpp::Time t0(p0.header.stamp, clock_type);
            const rclcpp::Time t1(p1.header.stamp, clock_type);
            const double dt = (t1 - t0).seconds();
            if (!(dt > 1e-3) || !std::isfinite(dt)) continue;

            const R2d dp(p1.vector.x - p0.vector.x, p1.vector.y - p0.vector.y);
            const R2d v = dp / dt;
            if (!v.array().isFinite().all()) continue;

            v_sum += v;
            const double h = std::atan2(v.y(), v.x());
            if (prev_heading_opt) heading_change_accum += std::fabs(wrapped_delta(h - *prev_heading_opt));
            prev_heading_opt = h;
            ++readings;
        }

        if (readings < 1) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Not enough valid velocity readings for drive-forward correction");
            return;
        }

        const R2d mean_v = v_sum / static_cast<double>(readings);
        double const speed = mean_v.norm();
        if (speed < min_linear_speed) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Drive-forward skipped: estimated planar speed %.3f m/s below minimum_linear_speed %.3f m/s",
                speed,
                min_linear_speed);
            return;
        }

        if (heading_change_accum > heading_delta_threshold) {
            return;
        }

        const double drive_forward_heading = std::atan2(mean_v.y(), mean_v.x());
        
        // for debugging purposes, publishing drive forward heading
        mrover::msg::Heading drive_forward_heading_msg;
        drive_forward_heading_msg.heading = std::fmod(90. - (drive_forward_heading * (180. / M_PI)) + 360., 360.);
        drive_forward_pub->publish(drive_forward_heading_msg);

        // Compare against current IMU-derived heading and do a Kalman correct on delta.
        auto const& qmsg = last_imu->orientation;
        if (!std::isfinite(qmsg.w) || !std::isfinite(qmsg.x) || !std::isfinite(qmsg.y) || !std::isfinite(qmsg.z)) {
            return;
        }
        Eigen::Quaterniond uncorrected_orientation(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
        const double norm2 = uncorrected_orientation.squaredNorm();
        if (!std::isfinite(norm2) || norm2 < 1e-12) {
            return;
        }
        uncorrected_orientation.normalize();
        const R2d uncorrected_forward = uncorrected_orientation.toRotationMatrix().col(0).head(2);
        if (!uncorrected_forward.array().isFinite().all()) {
            return;
        }
        const double uncorrected_heading = std::atan2(uncorrected_forward.y(), uncorrected_forward.x());
        if (!std::isfinite(uncorrected_heading)) {
            return;
        }

        auto const previousX = X;

        double heading_correction_delta = drive_forward_heading - uncorrected_heading;
        heading_correction_delta = wrapped_delta(heading_correction_delta);

        predict(process_noise);
        correct(heading_correction_delta, drive_noise);

        auto const correctionDelta = (X - previousX);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", std::format("Drive forward correction delta on X: {} rad", correctionDelta).c_str());
    }

}


int main(int argc, char**argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::HeadingFilter>());
    rclcpp::shutdown();
    return 0;

}
