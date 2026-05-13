#include "heading_filter.hpp"

namespace mrover {

    auto HeadingFilter::quat_geodesic_angle_rad(const Eigen::Quaterniond &prev_quat, const Eigen::Quaterniond &curr_quat) -> double {
        const Eigen::Quaterniond a_n = prev_quat.normalized();
        const Eigen::Quaterniond b_n = curr_quat.normalized();

        Eigen::Quaterniond q_rel = b_n * a_n.conjugate();
        if (q_rel.w() < 0) {
            q_rel.coeffs() *= -1.0;
        }

        q_rel.normalize();
        double half_angle = std::atan2(q_rel.vec().norm(), std::abs(q_rel.w()));
        return 2.0 * half_angle;
    }

    auto HeadingFilter::update_imu_derived_state(const sensor_msgs::msg::Imu& imu) -> bool {
        auto const& qmsg = imu.orientation;
        if (!std::isfinite(qmsg.w) || !std::isfinite(qmsg.x) || !std::isfinite(qmsg.y) || !std::isfinite(qmsg.z)) {
            RCLCPP_WARN(get_logger(), "IMU quaternion has a non-finite component, skipping IMU derived-state update");
            return false;
        }

        Eigen::Quaterniond q(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
        double const norm2 = q.squaredNorm();
        if (!std::isfinite(norm2) || norm2 < 1e-12) {
            RCLCPP_WARN(get_logger(), "IMU quaternion has invalid/near-zero norm, skipping IMU derived-state update");
            return false;
        }
        q.normalize();

        R2d const forward_xy = q.toRotationMatrix().col(0).head(2);
        if (!forward_xy.array().isFinite().all()) {
            RCLCPP_WARN(get_logger(), "IMU forward vector not finite, skipping IMU derived-state update");
            return false;
        }

        double const yaw_rad = std::atan2(forward_xy.y(), forward_xy.x());
        if (!std::isfinite(yaw_rad)) {
            RCLCPP_WARN(get_logger(), "Computed IMU yaw not finite, skipping IMU derived-state update");
            return false;
        }

        rclcpp::Time stamp{imu.header.stamp, get_clock()->get_clock_type()};
        if (stamp.nanoseconds() == 0) {
            stamp = get_clock()->now();
        }

        last_imu_derived = IMUOrientationState{
            .stamp = stamp,
            .orientation = SO3d(q),
            .forward_xy = forward_xy,
            .yaw_rad = yaw_rad,
        };
        last_imu = imu;
        return true;
    }

    HeadingFilter::HeadingFilter() : Node("heading_filter") {

        std::vector<ParameterWrapper> params{
            {"world_frame", world_frame, std::string("map")},
            {"gps_frame", gps_frame, std::string("gps_frame")},
            {"imu_watchdog_timeout", imu_timeout, 1.0},
            {"mag_heading_noise", mag_noise, 10.0},
            {"rtk_heading_noise", rtk_noise, 0.00001},
            {"drive_forward_heading_noise", drive_noise, 0.0001},
            {"process_noise", process_noise, 0.000001},
            {"minimum_linear_speed", min_speed, 0.4},
            {"rover_heading_change_threshold", heading_delta_threshold, 0.05},
            {"use_mag", use_mag, false},
        };

        ParameterWrapper::declareParameters(this, params);

        // Subscribers (State Estimation: ZED 2i IMU, RTK Dual-Antenna GPS; Manual Control Input: Joystick)

        linearized_position_sub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("/linearized_position", 1, [&](const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &position_msg) {
            last_position = *position_msg;
            position_window.push_back(*position_msg);
            if (position_window.size() > DRIVE_FORWARD_CAP) {
                position_window.pop_front();
            }
        });

        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, [&](const geometry_msgs::msg::Twist::ConstSharedPtr &twist_msg) {
            twists_window.push_back(*twist_msg);
            if (twists_window.size() > TWISTS_CAP) {
                twists_window.pop_front();
            }
        });

        rtk_heading_sub.subscribe(this, "/heading/fix");
        rtk_heading_status_sub.subscribe(this, "/heading_fix_status");
        imu_sub.subscribe(this, "/zed_imu/data_raw");
        mag_heading_sub.subscribe(this, "/zed_imu/mag_heading");

        // Synchronizers

        rtk_heading_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<mrover::msg::Heading, mrover::msg::FixStatus>>>(
            message_filters::sync_policies::ApproximateTime<mrover::msg::Heading, mrover::msg::FixStatus>(RTK_AND_IMU_YAW_SYNC_CAP),
            rtk_heading_sub,
            rtk_heading_status_sub
        );

        rtk_heading_sync->setAgePenalty(0.5);
        rtk_heading_sync->registerCallback(&HeadingFilter::sync_rtk_heading_callback, this);

        imu_and_mag_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, mrover::msg::Heading>>>(
            message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, mrover::msg::Heading>(RTK_AND_IMU_YAW_SYNC_CAP),
            imu_sub,
            mag_heading_sub
        );

        imu_and_mag_sync->setAgePenalty(0.5);
        imu_and_mag_sync->registerCallback(&HeadingFilter::sync_imu_and_mag_callback, this);

        // Publishers (Raw IMU-Derived Heading, Drive Forward (Instantaneous) Heading)

        drive_forward_heading_pub = this->create_publisher<mrover::msg::Heading>("/drive_forward_heading", 1);
        raw_imu_heading_pub = this->create_publisher<mrover::msg::Heading>("/imu_uncorrected_heading", 1);

        // IMU Watchdog (Helps to preserve the integrity of IMU data by catching bad values)

        const rclcpp::Duration IMU_WATCHDOG_TIMEOUT = rclcpp::Duration::from_seconds(imu_timeout);
        imu_and_mag_watchdog = this->create_wall_timer(
            IMU_WATCHDOG_TIMEOUT.to_chrono<std::chrono::milliseconds>(), 
            [&]() { RCLCPP_WARN(get_logger(), "ZED 2i IMU Data Watchdog Expired");
            last_imu.reset();
            last_imu_derived.reset();
            prev_imu_orientation_norm.reset();
            imu_unstuck_counter = 0;
            imu_stuck_counter = 0;
            imu_orientation_stuck = false;
        });

        // Wall Timers

        drive_forward_timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(DRIVE_FORWARD_TIMER * 1000.0)),
            [this]() -> void { drive_forward_callback(); 
        });

        tf_publish_timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(ROVER_POSE_TIMER * 1000.0)),
            [this]() -> void { publish_tf_callback(); }
        );

        // Initial Kalman Params (State Estimate & Estimate Uncertainty)

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
        if (heading_status->fix_type.fix != mrover::msg::FixType::FIXED) {
            return;
        }

        double const rover_map_deg = std::fmod(heading->heading + 90. + 360., 360.);
        double measured_heading_deg = 90. - rover_map_deg;
        if (measured_heading_deg <= -180.) {
            measured_heading_deg += 360.;
        } else if (measured_heading_deg > 180.) {
            measured_heading_deg -= 360.;
        }
        double const measured_heading = measured_heading_deg * (M_PI / 180.);
        last_rtk_yaw = {get_clock()->now(), measured_heading};

        if (!last_imu_derived) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No cached IMU-derived state for RTK correction");
            return;
        }

        auto const previousX = X;
        double heading_correction_delta = measured_heading - last_imu_derived->yaw_rad;
        heading_correction_delta = std::fmod((heading_correction_delta + 3 * M_PI), 2 * M_PI) - M_PI;

        predict(process_noise);
        correct(heading_correction_delta, rtk_noise);

        auto const correctionDelta = (X - previousX);
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "%s",
            std::format("RTK heading correction delta on X: {} rad", correctionDelta).c_str());
    }

    void HeadingFilter::sync_imu_and_mag_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu, const mrover::msg::Heading::ConstSharedPtr &mag_heading) {
        imu_and_mag_watchdog.reset();
        if (!update_imu_derived_state(*imu) || !last_imu_derived) {
            return;
        }

        if (!last_position) {
            RCLCPP_WARN(get_logger(), "No position data!");
            return;
        }

        auto const& imu_state = *last_imu_derived;
        Eigen::Quaterniond const uncorrected_orientation = imu_state.orientation.quat().normalized();

        R3d position_in_map(last_position->vector.x, last_position->vector.y, last_position->vector.z);
        SE3d pose_in_map(position_in_map, SO3d::Identity());

        mrover::msg::Heading imu_heading_msg;
        imu_heading_msg.heading = std::fmod(90. - (imu_state.yaw_rad * (180. / M_PI)) + 360., 360.);
        raw_imu_heading_pub->publish(imu_heading_msg);

        if (prev_imu_orientation_norm) {
            double const theta = quat_geodesic_angle_rad(*prev_imu_orientation_norm, uncorrected_orientation);
            if (theta < EPS_STALE) {
                ++imu_stuck_counter;
                imu_unstuck_counter = 0;
            } else {
                ++imu_unstuck_counter;
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
        } else if (imu_stuck_counter >= IMU_STUCK_THRESHOLD) {
            imu_orientation_stuck = true;
            imu_stuck_counter = 0;
            RCLCPP_WARN(get_logger(), "IMU orientation appears stuck (constant quaternion)");
        }

        prev_imu_orientation_norm = uncorrected_orientation;

        if (use_mag) {
            double measured_heading_deg = 90. - mag_heading->heading;
            if (measured_heading_deg <= -180.) {
                measured_heading_deg += 360.;
            } else if (measured_heading_deg > 180.) {
                measured_heading_deg -= 360.;
            }
            double const measured_heading = measured_heading_deg * (M_PI / 180.);

            auto const previousX = X;
            double heading_correction_delta = measured_heading - imu_state.yaw_rad;
            heading_correction_delta = std::fmod((heading_correction_delta + 3 * M_PI), 2 * M_PI) - M_PI;

            predict(process_noise);
            correct(heading_correction_delta, mag_noise);

            auto const correctionDelta = (X - previousX);
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "%s",
                std::format("Mag heading correction delta on X: {} rad", correctionDelta).c_str());
        }

    }

    void HeadingFilter::drive_forward_callback() {
        if (!last_imu_derived) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No cached IMU-derived state for drive-forward correction");
            return;
        }

        double const min_linear_speed = min_speed;

        // Gate on "commanded forward"
        bool const had_cmd_vel_samples = !twists_window.empty();
        double mean_cmd_vel = 0.0;
        if (had_cmd_vel_samples) {
            for (auto const& twist : twists_window) {
                mean_cmd_vel += twist.linear.x / static_cast<double>(twists_window.size());
            }
        }
        twists_window.clear();
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
        drive_forward_heading_pub->publish(drive_forward_heading_msg);
        last_drive_forward_yaw = {get_clock()->now(), drive_forward_heading};

        auto const previousX = X;
        double heading_correction_delta = drive_forward_heading - last_imu_derived->yaw_rad;
        heading_correction_delta = wrapped_delta(heading_correction_delta);

        predict(process_noise);
        correct(heading_correction_delta, drive_noise);

        auto const correctionDelta = (X - previousX);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", std::format("Drive forward correction delta on X: {} rad", correctionDelta).c_str());
    }

    void HeadingFilter::publish_tf_callback() {
        if (!last_position) {
            return;
        }
        if (!std::isfinite(X)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Kalman state X not finite, skipping TF publish");
            return;
        }

        auto wrap_pm_pi = [](double a) -> double {
            return std::fmod(a + 3.0 * M_PI, 2.0 * M_PI) - M_PI;
        };

        auto const now = get_clock()->now();
        auto is_fresh = [&](rclcpp::Time const& stamp, double max_age_s) -> bool {
            return (now - stamp).seconds() <= max_age_s;
        };

        bool const imu_fresh = last_imu_derived.has_value() && is_fresh(last_imu_derived->stamp, imu_timeout);
        bool const use_full_imu_orientation = imu_fresh && !imu_orientation_stuck;
        double const yaw_source_max_age_s = 0.5;

        R3d position_in_map(last_position->vector.x, last_position->vector.y, last_position->vector.z);
        SO3d published_orientation = SO3d::Identity();

        if (use_full_imu_orientation) {
            SO3d const curr_heading_correction = Eigen::AngleAxisd(X, R3d::UnitZ());
            SO3d const corrected_orientation = curr_heading_correction * last_imu_derived->orientation;
            Eigen::Quaterniond q = corrected_orientation.quat();
            q.normalize();
            if (!q.coeffs().array().isFinite().all() || q.squaredNorm() < 1e-12) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Corrected orientation quaternion invalid, skipping TF publish");
                return;
            }
            published_orientation = SO3d(q);
        } else {
            if (imu_orientation_stuck) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "IMU orientation stuck: publishing yaw-only orientation");
            }
            std::optional<double> yaw_base_rad;
            if (last_rtk_yaw && is_fresh(last_rtk_yaw->first, yaw_source_max_age_s)) {
                yaw_base_rad = last_rtk_yaw->second;
            } else if (last_drive_forward_yaw && is_fresh(last_drive_forward_yaw->first, yaw_source_max_age_s)) {
                yaw_base_rad = last_drive_forward_yaw->second;
            } else if (last_imu_derived) {
                yaw_base_rad = last_imu_derived->yaw_rad;
            }

            if (!yaw_base_rad) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No yaw source available for degraded TF publish");
                return;
            }

            double const yaw_pub = wrap_pm_pi(*yaw_base_rad + X);
            published_orientation = SO3d(Eigen::AngleAxisd(yaw_pub, R3d::UnitZ()));
        }

        SE3d pose_in_map(position_in_map, published_orientation);
        SE3Conversions::pushToTfTree(tf_broadcaster, gps_frame, world_frame, pose_in_map, get_clock()->now());
    }

}