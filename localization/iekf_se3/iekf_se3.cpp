#include "iekf_se3.hpp"

namespace mrover {
    
    IEKF_SE3::IEKF_SE3() : Node{"iekf_se3"} {

        // declare ros params
        declare_parameter("world_frame", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("rover_frame", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("scale_cov_a", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("scale_cov_w", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("pos_noise_fixed", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("vel_noise", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("mag_heading_noise", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("rtk_heading_noise", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("drive_forward_heading_noise", rclcpp::ParameterType::PARAMETER_DOUBLE);

        declare_parameter("rover_heading_change_threshold", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("minimum_linear_speed", rclcpp::ParameterType::PARAMETER_DOUBLE);
        
        declare_parameter("use_mag", rclcpp::ParameterType::PARAMETER_BOOL);
        declare_parameter("use_drive_forward", rclcpp::ParameterType::PARAMETER_BOOL);
    
        world_frame = get_parameter("world_frame").as_string();
        rover_frame = get_parameter("rover_frame").as_string();
        scale_cov_a = get_parameter("scale_cov_a").as_double();
        scale_cov_w = get_parameter("scale_cov_w").as_double();
        pos_noise_fixed = get_parameter("pos_noise_fixed").as_double();
        vel_noise = get_parameter("vel_noise").as_double();
        mag_heading_noise = get_parameter("mag_heading_noise").as_double();
        rtk_heading_noise = get_parameter("rtk_heading_noise").as_double();
        drive_forward_heading_noise = get_parameter("drive_forward_heading_noise").as_double();

        rover_heading_change_threshold = get_parameter("rover_heading_change_threshold").as_double();
        minimum_linear_speed = get_parameter("minimum_linear_speed").as_double();

        use_mag = get_parameter("use_mag").as_bool();
        use_drive_forward = get_parameter("use_drive_forward").as_bool();

        // initialize state variables
        X.setIdentity();
        P.setIdentity();
        A.setZero();

        // subscribers
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/zed_imu/data_raw", 10, [&](const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg) {
            imu_callback(*imu_msg);
        });

        mag_heading_sub = this->create_subscription<mrover::msg::Heading>("/zed_imu/mag_heading", 10, [&](const mrover::msg::Heading::ConstSharedPtr& mag_heading_msg) {
            mag_heading_callback(*mag_heading_msg);
        });

        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, [&](const geometry_msgs::msg::Twist::ConstSharedPtr& cmd_vel_msg) {
            twists.push_back(*cmd_vel_msg);
        });

        pos_sub.subscribe(this, "/linearized_position");
        pos_status_sub.subscribe(this, "/gps_fix_status");

        pos_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::Vector3Stamped, mrover::msg::FixStatus>>>(
            message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::Vector3Stamped, mrover::msg::FixStatus>(10),
            pos_sub,
            pos_status_sub
        );

        pos_sync->setAgePenalty(0.5);
        pos_sync->registerCallback(&IEKF_SE3::pos_callback, this);


        velocity_sub.subscribe(this, "/velocity/fix");
        velocity_status_sub.subscribe(this, "/velocity_fix_status");

        velocity_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::Vector3Stamped, mrover::msg::FixStatus>>>(
            message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::Vector3Stamped, mrover::msg::FixStatus>(10),
            velocity_sub,
            velocity_status_sub
        );

        velocity_sync->setAgePenalty(0.5);
        velocity_sync->registerCallback(&IEKF_SE3::vel_callback, this);


        rtk_heading_sub.subscribe(this, "/heading/fix");
        rtk_heading_status_sub.subscribe(this, "/heading_fix_status");

        rtk_heading_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<mrover::msg::Heading, mrover::msg::FixStatus>>>(
            message_filters::sync_policies::ApproximateTime<mrover::msg::Heading, mrover::msg::FixStatus>(10),
            rtk_heading_sub,
            rtk_heading_status_sub
        );

        rtk_heading_sync->setAgePenalty(0.5);
        rtk_heading_sync->registerCallback(&IEKF_SE3::rtk_heading_callback, this);

        correction_timer = this->create_wall_timer(WINDOW.to_chrono<std::chrono::milliseconds>(), [this]() -> void {
            drive_forward_callback();
        });

    }


    auto IEKF_SE3::adjoint() -> Matrix66d {

        Matrix66d adj = Matrix66d::Zero();

        Matrix33d p_skew = manif::skew(X.col(3).head(3));

        adj.block<3, 3>(0, 0) = X.block<3, 3>(0, 0);
        adj.block<3, 3>(3, 0) = p_skew * X.block<3, 3>(0, 0);
        adj.block<3, 3>(3, 3) = X.block<3, 3>(0, 0);

        return adj;

    }

    auto IEKF_SE3::lift(const Vector6d& dx) -> Matrix44d {
        
        Matrix44d result = Matrix44d::Zero();
        result.block<3, 3>(0, 0) = manif::skew(dx.head(3));
        result(0, 3) = dx(3);
        result(1, 3) = dx(4);
        result(2, 3) = dx(5);

        return result;
    }

    void IEKF_SE3::predict_vel(const Vector3d& v, const Matrix33d& cov_v, double dt) {

        Matrix66d adj_x = adjoint();
        Matrix66d Q = Matrix66d::Zero();
        Matrix66d Q_d = Matrix66d::Zero();

        A << Matrix33d::Zero(), Matrix33d::Zero(), manif::skew(v), Matrix33d::Zero();

        // covariance
        Q.block<3, 3>(3, 3) = cov_v.cwiseAbs();
        Q_d = (A * dt).exp() * Q * dt * ((A * dt).exp()).transpose();

        // propagate
        X.block<3, 1>(0, 3) =  X.block<3, 1>(0, 3) + v * dt;
        P = (A * dt).exp() * P * ((A * dt).exp()).transpose() + adj_x * Q_d * adj_x.transpose();

    }


    void IEKF_SE3::predict_imu(const Vector3d& w, const Matrix33d& cov_w, double dt) {

        Matrix66d adj_x = adjoint();
        Matrix66d Q = Matrix66d::Zero();
        Matrix66d Q_d = Matrix66d::Zero();

        // covariance
        Q.block<3, 3>(0, 0) = cov_w.cwiseAbs();
        Q_d = (A * dt).exp() * Q * dt * ((A * dt).exp()).transpose();

        // propagate
        X.block<3, 3>(0, 0) = X.block<3, 3>(0, 0) * (manif::skew(w) * dt).exp();
        P = (A * dt).exp() * P * ((A * dt).exp()).transpose() + adj_x * Q_d * adj_x.transpose();
        
    }


    void IEKF_SE3::correct(const Vector4d& Y, const Vector4d& b, const Matrix33d& N, const Matrix36d& H) {

        Vector4d innov = X * Y - b;
        Matrix33d S = H * P * H.transpose() + N;
        Matrix63d L = P * H.transpose() * S.inverse();
        Matrix44d dx = lift(L * innov.head(3));
                                          
        X = dx.exp() * X;
        P = (Matrix66d::Identity() - L * H) * P * (Matrix66d::Identity() - L * H).transpose() + L * N * L.transpose();
       
    }


    void IEKF_SE3::imu_callback(const sensor_msgs::msg::Imu& imu_msg) {

        geometry_msgs::msg::Vector3 w = imu_msg.angular_velocity;
        Matrix33d cov_w;
        cov_w << imu_msg.angular_velocity_covariance[0], imu_msg.angular_velocity_covariance[1], imu_msg.angular_velocity_covariance[2],
                 imu_msg.angular_velocity_covariance[3], imu_msg.angular_velocity_covariance[4], imu_msg.angular_velocity_covariance[5],
                 imu_msg.angular_velocity_covariance[6], imu_msg.angular_velocity_covariance[7], imu_msg.angular_velocity_covariance[8];

        geometry_msgs::msg::Vector3 a = imu_msg.linear_acceleration;
        Matrix33d cov_a;
        cov_a << imu_msg.linear_acceleration_covariance[0], imu_msg.linear_acceleration_covariance[1], imu_msg.linear_acceleration_covariance[2],
                 imu_msg.linear_acceleration_covariance[3], imu_msg.linear_acceleration_covariance[4], imu_msg.linear_acceleration_covariance[5],
                 imu_msg.linear_acceleration_covariance[6], imu_msg.linear_acceleration_covariance[7], imu_msg.linear_acceleration_covariance[8];
        
        double dt = IMU_DT;
        if (last_imu_time) {
            dt = (imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9) - (last_imu_time.value().sec + last_imu_time.value().nanosec * 1e-9);   
        }

        predict_imu(Vector3d{w.x, w.y, w.z}, scale_cov_w * cov_w, dt);
        
        accel_callback(a, scale_cov_a * cov_a);

        R3d translation = X.block<3, 1>(0, 3);
        SO3d rotation = Eigen::Quaterniond(X.block<3, 3>(0, 0));

        SE3d pose_in_map(translation, rotation);
        SE3Conversions::pushToTfTree(tf_broadcaster, rover_frame, world_frame, pose_in_map, get_clock()->now());

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Published SE3 to TF tree: translation (%f, %f, %f) orientation (%f, %f, %f, %f)", translation.x(), translation.y(), translation.z(), pose_in_map.quat().x(), pose_in_map.quat().y(), pose_in_map.quat().z(), pose_in_map.quat().w());

        last_imu_time = imu_msg.header.stamp;

    }

    void IEKF_SE3::vel_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& vel_msg, const mrover::msg::FixStatus::ConstSharedPtr& vel_status_msg) {

        if (vel_status_msg->fix_type.fix == mrover::msg::FixType::NO_SOL) {
            return;
        }

        Matrix33d cov_v = vel_noise * Matrix33d::Identity();

        double dt = VEL_DT;
        if (last_vel_time) {
            dt = (vel_msg->header.stamp.sec + vel_msg->header.stamp.nanosec * 1e-9) - (last_vel_time.value().sec + last_vel_time.value().nanosec * 1e-9);
        }
        predict_vel(Vector3d{vel_msg->vector.x, vel_msg->vector.y, vel_msg->vector.z}, cov_v, dt);
        
        last_vel_time = vel_msg->header.stamp;

       
    }


    void IEKF_SE3::pos_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& pos_msg, const mrover::msg::FixStatus::ConstSharedPtr& pos_status_msg) {

        if (pos_status_msg->fix_type.fix == mrover::msg::FixType::NO_SOL) {
            return;
        }

        Matrix36d H;
        Vector4d Y;
        Vector4d b;
        Matrix33d N;

        H << Matrix33d::Zero(), -1 * Matrix33d::Identity();
        Y << -1 * X.block<3, 3>(0, 0).transpose() * Vector3d{pos_msg->vector.x, pos_msg->vector.y, pos_msg->vector.z}, 1;
        b << 0, 0, 0, 1;
        N << pos_noise_fixed * Matrix33d::Identity();

        correct(Y, b, N, H);
        
    }
    

    void IEKF_SE3::mag_heading_callback(const mrover::msg::Heading& mag_heading_msg) {

        Matrix36d H;
        Vector4d Y;
        Vector4d b;
        Matrix33d N;

        Vector3d mag_ref{1, 0, 0};

        double heading = 90 - mag_heading_msg.heading;

        if (heading < -180) {
            heading = 360 + heading;
        }

        heading = heading * (M_PI / 180);

        H << -1 * manif::skew(mag_ref), Matrix33d::Zero();
        Y << -1 * Vector3d{std::cos(heading), -std::sin(heading), 0}, 0;
        b << -1 * mag_ref, 0;
        N << X.block<3, 3>(0, 0) * mag_heading_noise * Matrix33d::Identity() * X.block<3, 3>(0, 0).transpose();

        if (use_mag) {
            correct(Y, b, N, H);
        }
    
    }
    

    void IEKF_SE3::accel_callback(const geometry_msgs::msg::Vector3 &a, const Matrix33d &cov_a) {

        Matrix36d H;
        Vector4d Y;
        Vector4d b;
        Matrix33d N;

        Vector3d accel_ref{0, 0, 1};

        Vector3d accel_meas{a.x, a.y, a.z};
        accel_meas.normalize();

        H << -1 * manif::skew(accel_ref), Matrix33d::Zero();
        Y << -1 * accel_meas, 0;
        b << -1 * accel_ref, 0;
        N << X.block<3, 3>(0, 0) * cov_a * X.block<3, 3>(0, 0).transpose();

        correct(Y, b, N, H);
    }

    void IEKF_SE3::rtk_heading_callback(const mrover::msg::Heading::ConstSharedPtr &rtk_heading, const mrover::msg::FixStatus::ConstSharedPtr &rtk_heading_status) {

        if (rtk_heading_status->fix_type.fix == mrover::msg::FixType::NO_SOL) {
            return;
        }

        Matrix36d H;
        Vector4d Y;
        Vector4d b;
        Matrix33d N;

        Vector3d rtk_heading_ref{1, 0, 0};

        double heading = 90 - fmod(rtk_heading->heading + 90, 360);

        if (heading < -180) {
            heading = 360 + heading;
        }

        heading = heading * (M_PI / 180);

        H << -1 * manif::skew(rtk_heading_ref), Matrix33d::Zero();
        Y << -1 * Vector3d{std::cos(heading), -std::sin(heading), 0}, 0;
        b << -1 * rtk_heading_ref, 0;
        N << X.block<3, 3>(0, 0) * rtk_heading_noise * Matrix33d::Identity() * X.block<3, 3>(0, 0).transpose();

        correct(Y, b, N, H);

    }

    void IEKF_SE3::drive_forward_callback() {

        double mean_cmd_vel = 0;
        for (auto & twist : twists) {
            mean_cmd_vel += twist.linear.x / static_cast<double>(twists.size());
        }
        twists.clear();
        if (mean_cmd_vel < minimum_linear_speed) {
            RCLCPP_WARN(get_logger(), "Rover is not being commanded forward!");
            return;
        }

        R2d rover_velocity_sum = R2d::Zero();
        double rover_heading_old = 0.0;
        double rover_heading_new = 0.0;
        double rover_heading_change = 0.0;
        std::size_t readings = 0;

        rclcpp::Time end = get_clock()->now();
        rclcpp::Time start = end - WINDOW;

        for (rclcpp::Time t = start; t < end; t += STEP) {

            try {
                auto rover_in_map_old = SE3Conversions::fromTfTree(tf_buffer, rover_frame, world_frame, t - STEP);
                auto rover_in_map_new = SE3Conversions::fromTfTree(tf_buffer, rover_frame, world_frame, t);
                R3d rover_velocity_in_map = (rover_in_map_new.translation() - rover_in_map_old.translation()) / STEP.seconds();

                if (t == start) {
                    rover_heading_old = std::atan2(rover_velocity_in_map(1), rover_velocity_in_map(0));
                    rover_heading_new = rover_heading_old;
                }
                else {
                    rover_heading_new = std::atan2(rover_velocity_in_map(1), rover_velocity_in_map(0));
                }

                rover_velocity_sum += rover_velocity_in_map.head<2>();
                rover_heading_change += std::fabs(rover_heading_new - rover_heading_old);
                rover_heading_old = rover_heading_new;
                ++readings;
            } catch (tf2::ConnectivityException const& e) {
                RCLCPP_WARN_STREAM(get_logger(), e.what());
                return;
            } catch (tf2::LookupException const& e) {
                RCLCPP_WARN_STREAM(get_logger(), e.what());
                return;
            } catch (tf2::ExtrapolationException const& e) {
                RCLCPP_WARN_STREAM(get_logger(), e.what());
                return;
            }
        }

        if (rover_heading_change > rover_heading_change_threshold) {
            RCLCPP_WARN(get_logger(), "Rover is not moving straight enough: Heading change = {%f} deg", rover_heading_change * (180 / M_PI));
            return;
        }

        R2d mean_velocity = rover_velocity_sum / static_cast<double>(readings);
        if (mean_velocity.norm() < minimum_linear_speed) {
            RCLCPP_WARN(get_logger(), "Rover stationary - insufficient speed: %.3f m/s", mean_velocity.norm());
            return;
        }
        
        double drive_forward_heading = atan2(rover_velocity_sum.y(), rover_velocity_sum.x());

        RCLCPP_INFO(get_logger(), "Drive forward heading: %f deg", drive_forward_heading * (180 / M_PI));

        Matrix36d H;
        Vector4d Y;
        Vector4d b;
        Matrix33d N;

        Vector3d drive_forward_ref{1, 0, 0};

        H << -1 * manif::skew(drive_forward_ref), Matrix33d::Zero();
        Y << -1 * Vector3d{std::cos(drive_forward_heading), -std::sin(drive_forward_heading), 0}, 0;
        b << -1 * drive_forward_ref, 0;
        N << X.block<3, 3>(0, 0) * drive_forward_heading_noise * Matrix33d::Identity() * X.block<3, 3>(0, 0).transpose();

        if (use_drive_forward) {
            correct(Y, b, N, H);
        }
        
    }


}

int main(int argc, char**argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::IEKF_SE3>());
    rclcpp::shutdown();
    return 0;

}