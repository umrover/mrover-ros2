#include "iekf_se3.hpp"

namespace mrover {
    
    IEKF_SE3::IEKF_SE3() : Node{"iekf_se3"} {

        // declare ros params
        declare_parameter("world_frame", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("rover_frame", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("scale_cov_a", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("scale_cov_w", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("pos_noise_fixed", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("pos_noise_float", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("pos_noise_none", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("vel_noise", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("mag_heading_noise", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("rtk_heading_noise", rclcpp::ParameterType::PARAMETER_DOUBLE);

        declare_parameter("rover_heading_change_threshold", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("minimum_linear_speed", rclcpp::ParameterType::PARAMETER_DOUBLE);
    
        world_frame = get_parameter("world_frame").as_string();
        rover_frame = get_parameter("rover_frame").as_string();
        scale_cov_a = get_parameter("scale_cov_a").as_double();
        scale_cov_w = get_parameter("scale_cov_w").as_double();
        pos_noise_fixed = get_parameter("pos_noise_fixed").as_double();
        vel_noise = get_parameter("vel_noise").as_double();
        mag_heading_noise = get_parameter("mag_heading_noise").as_double();

        rover_heading_change_threshold = get_parameter("rover_heading_change_threshold").as_double();
        minimum_linear_speed = get_parameter("minimum_linear_speed").as_double();

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

        pos_sub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("/linearized_position", 10, [&](const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& pos_msg) {
            pos_callback(pos_msg->vector);
        });

        velocity_sub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("/velocity/fix", 10, [&](const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& vel_msg) {
            vel_callback(*vel_msg);
        });

        correction_timer = this->create_wall_timer(WINDOW.to_chrono<std::chrono::milliseconds>(), [this]() -> void {
            drive_forward_callback();
        });

        // subscribers sim
        // imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, [&](const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg) {
        //     imu_callback_sim(*imu_msg);
        // });

        // mag_heading_sub = this->create_subscription<mrover::msg::Heading>("/imu/mag", 10, [&](const mrover::msg::Heading::ConstSharedPtr& mag_heading_msg) {
        //     mag_heading_callback_sim(*mag_heading_msg);
        // });


    }


    auto IEKF_SE3::adjoint() -> Matrix66d {

        Matrix66d adj = Matrix66d::Zero();

        Matrix33d p_skew = manif::skew(X.col(3).head(3));

        adj.block<3, 3>(0, 0) = X.block<3, 3>(0, 0);
        adj.block<3, 3>(3, 0) = p_skew * X.block<3, 3>(0, 0);
        adj.block<3, 3>(3, 3) = X.block<3, 3>(0, 0);

        return adj;

    }

    auto lift(const Vector6d& dx) -> Matrix44d {
        
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

        RCLCPP_INFO(get_logger(), "X after:\n%f, %f, %f, %f\n%f, %f, %f, %f\n %f, %f, %f, %f\n%f, %f, %f, %f", X(0,0), X(0,1), X(0,2), X(0, 3),
                                                                                 X(1,0), X(1,1), X(1,2), X(1, 3),
                                                                                 X(2,0), X(2,1), X(2,2), X(2, 3),
                                                                                 X(3,0), X(3,1), X(3,2), X(3, 3));
       
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

        last_imu_time = imu_msg.header.stamp;

    }

    void IEKF_SE3::vel_callback(const geometry_msgs::msg::Vector3Stamped& vel_msg) {

        Matrix33d cov_v = vel_noise * Matrix33d::Identity();

        double dt = VEL_DT;
        if (last_vel_time) {
            dt = (vel_msg.header.stamp.sec + vel_msg.header.stamp.nanosec * 1e-9) - (last_vel_time.value().sec + last_vel_time.value().nanosec * 1e-9);
        }
        predict_vel(Vector3d{vel_msg.vector.x, vel_msg.vector.y, vel_msg.vector.z}, cov_v, dt);
        
        last_vel_time = vel_msg.header.stamp;

       
    }


    void IEKF_SE3::pos_callback(const geometry_msgs::msg::Vector3& pos_msg) {

        RCLCPP_INFO(get_logger(), "pos callback");

        Matrix36d H;
        Vector4d Y;
        Vector4d b;
        Matrix33d N;

        H << Matrix33d::Zero(), -1 * Matrix33d::Identity();
        Y << -1 * X.block<3, 3>(0, 0).transpose() * Vector3d{pos_msg.x, pos_msg.y, pos_msg.z}, 1;
        b << 0, 0, 0, 1;
        N << pos_noise_fixed * Matrix33d::Identity();

        correct(Y, b, N, H);

        // geometry_msgs::msg::Vector3 vel_msg;
        // vel_msg.x = 0;
        // vel_msg.y = 0;
        // vel_msg.z = 0;
        // vel_callback(vel_msg);
        
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

        RCLCPP_INFO(get_logger(), "heading: %f", heading);
        correct(Y, b, N, H);
    
    }
    

    void IEKF_SE3::accel_callback(const geometry_msgs::msg::Vector3 &a, const Matrix33d &cov_a) {

        Matrix36d H;
        Vector4d Y;
        Vector4d b;
        Matrix33d N;

        Vector3d accel_ref{0, 0, 1};

        Vector3d accel_meas{a.x, a.y, a.z};
        accel_meas.normalize();

        RCLCPP_INFO(get_logger(), "accel measured: %f, %f, %f", accel_meas(0), accel_meas(1), accel_meas(2));

        H << -1 * manif::skew(accel_ref), Matrix33d::Zero();
        Y << -1 * accel_meas, 0;
        b << -1 * accel_ref, 0;
        N << X.block<3, 3>(0, 0) * cov_a * X.block<3, 3>(0, 0).transpose();

        correct(Y, b, N, H);
    }

    void IEKF_SE3::drive_forward_callback() {

        R2d rover_velocity_sum = R2d::Zero();
        double rover_heading_change = 0.0;
        std::size_t readings = 0;

        rclcpp::Time end = get_clock()->now();
        rclcpp::Time start = end - WINDOW;

        for (rclcpp::Time t = start; t < end; t += STEP) {

            try {
                auto rover_in_map_old = SE3Conversions::fromTfTree(tf_buffer, rover_frame, world_frame, t - STEP);
                auto rover_in_map_new = SE3Conversions::fromTfTree(tf_buffer, rover_frame, world_frame, t);
                R3d rover_velocity_in_map = (rover_in_map_new.translation() - rover_in_map_old.translation()) / STEP.seconds();
                R3d rover_angular_velocity_in_map = (rover_in_map_new.asSO3() - rover_in_map_old.asSO3()).coeffs();
                rover_velocity_sum += rover_velocity_in_map.head<2>();
                rover_heading_change += std::fabs(rover_angular_velocity_in_map.z());
                ++readings;
            } catch (tf2::ConnectivityException const& e) {
                RCLCPP_WARN_STREAM(get_logger(), e.what());
                return;
            } catch (tf2::LookupException const& e) {
                RCLCPP_WARN_STREAM(get_logger(), e.what());
                return;
            } catch (tf2::ExtrapolationException const&) { }
        }


        if (rover_heading_change > rover_heading_change_threshold) {
            RCLCPP_WARN(get_logger(), "Rover is not moving straight enough: Heading change = {%f} rad", rover_heading_change);
            return;
        }

        R2d mean_velocity = rover_velocity_sum / static_cast<double>(readings);
        if (mean_velocity.norm() < minimum_linear_speed) {
            RCLCPP_WARN(this->get_logger(), "Rover stationary - insufficient speed: %.3f m/s", mean_velocity.norm());
            return;
        }
        
        double drive_forward_heading = atan2(rover_velocity_sum.y(), rover_velocity_sum.x());

        RCLCPP_INFO(get_logger(), "Drive forward heading: %f", drive_forward_heading);
        
    }


    // void IEKF_SE3::predict_sim(const Vector3d& w, const Matrix33d& cov_w, const Vector3d& a, const Matrix33d& cov_a, double dt) {
    //     Matrix99d adj_x = adjoint();
    //     Matrix99d Q = Matrix99d::Zero();
    //     Matrix99d Q_d = Matrix99d::Zero();

    //     // process noise
    //     Q.block(0, 0, 3, 3) = cov_w.cwiseAbs();
    //     Q.block(3, 3, 3, 3) = cov_a.cwiseAbs();
    //     Q.block(6, 6, 3, 3) = cov_a.cwiseAbs() * dt;
    //     Q_d = (A * dt).exp() * Q * dt * ((A * dt).exp()).transpose();

    //     // propagate
    //     X.block<3, 3>(0, 0) = X.block<3, 3>(0, 0) * (manif::skew(w) * dt).exp();
    //     X.block<3, 1>(0, 4) = X.block<3, 1>(0, 4) + X.block<3, 1>(0, 3) * dt + 0.5 * a * pow(dt, 2);
    //     X.block<3, 1>(0, 3) = X.block<3, 1>(0, 3) + a * dt;
        
    //     P = (A * dt).exp() * P * ((A * dt).exp()).transpose() + adj_x * Q_d * adj_x.transpose();
        
    //     // RCLCPP_INFO(get_logger(), "P:\n%f, %f, %f, %f, %f, %f, %f, %f, %f\n%f, %f, %f, %f, %f, %f, %f, %f, %f\n%f, %f, %f, %f, %f, %f, %f, %f, %f\n%f, %f, %f, %f, %f, %f, %f, %f, %f\n%f, %f, %f, %f, %f, %f, %f, %f, %f\n%f, %f, %f, %f, %f, %f, %f, %f, %f\n%f, %f, %f, %f, %f, %f, %f, %f, %f\n%f, %f, %f, %f, %f, %f, %f, %f, %f\n%f, %f, %f, %f, %f, %f, %f, %f, %f", 
    //     //                                     P(0,0), P(0,1), P(0,2), P(0, 3), P(0, 4), P(0, 5), P(0, 6), P(0, 7), P(0, 8),
    //     //                                     P(1,0), P(1,1), P(1,2), P(1, 3), P(1, 4), P(1, 5), P(1, 6), P(1, 7), P(1, 8),
    //     //                                     P(2,0), P(2,1), P(2,2), P(2, 3), P(2, 4), P(2, 5), P(2, 6), P(2, 7), P(2, 8),
    //     //                                     P(3,0), P(3,1), P(3,2), P(3, 3), P(3, 4), P(3, 5), P(3, 6), P(3, 7), P(3, 8),
    //     //                                     P(4,0), P(4,1), P(4,2), P(4, 3), P(4, 4), P(4, 5), P(4, 6), P(4, 7), P(4, 8),
    //     //                                     P(5,0), P(5,1), P(5,2), P(5, 3), P(5, 4), P(5, 5), P(5, 6), P(5, 7), P(5, 8),
    //     //                                     P(6,0), P(6,1), P(6,2), P(6, 3), P(6, 4), P(6, 5), P(6, 6), P(6, 7), P(6, 8),
    //     //                                     P(7,0), P(7,1), P(7,2), P(7, 3), P(7, 4), P(7, 5), P(7, 6), P(7, 7), P(7, 8),
    //     //                                     P(8,0), P(8,1), P(8,2), P(8, 3), P(8, 4), P(8, 5), P(8, 6), P(8, 7), P(8, 8));
                                                                                 

    //     // Matrix33d rot_m = X.block<3, 3>(0, 0);
    //     // RCLCPP_INFO(get_logger(), "\n%f, %f, %f\n%f, %f, %f\n %f, %f, %f", rot_m(0,0), rot_m(0,1), rot_m(0,2),
    //     //                                                                          rot_m(1,0), rot_m(1,1), rot_m(1,2),
    //     //                                                                          rot_m(2,0), rot_m(2,1), rot_m(2,2));


    // }

    // void IEKF_SE3::imu_callback_sim(const sensor_msgs::msg::Imu& imu_msg) {

    //     geometry_msgs::msg::Vector3 w = imu_msg.angular_velocity;
    //     Matrix33d cov_w;
    //     cov_w << imu_msg.angular_velocity_covariance[0], imu_msg.angular_velocity_covariance[1], imu_msg.angular_velocity_covariance[2],
    //              imu_msg.angular_velocity_covariance[3], imu_msg.angular_velocity_covariance[4], imu_msg.angular_velocity_covariance[5],
    //              imu_msg.angular_velocity_covariance[6], imu_msg.angular_velocity_covariance[7], imu_msg.angular_velocity_covariance[8];

    //     geometry_msgs::msg::Vector3 a = imu_msg.linear_acceleration;
    //     Matrix33d cov_a;
    //     cov_a << imu_msg.linear_acceleration_covariance[0], imu_msg.linear_acceleration_covariance[1], imu_msg.linear_acceleration_covariance[2],
    //              imu_msg.linear_acceleration_covariance[3], imu_msg.linear_acceleration_covariance[4], imu_msg.linear_acceleration_covariance[5],
    //              imu_msg.linear_acceleration_covariance[6], imu_msg.linear_acceleration_covariance[7], imu_msg.linear_acceleration_covariance[8];
        
    //     double dt = IMU_DT;
    //     if (last_imu_time) {
    //         dt = (imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9) - (last_imu_time.value().sec + last_imu_time.value().nanosec * 1e-9);   
    //     }

    //     predict_sim(Vector3d{w.x, w.y, w.z}, cov_w, Vector3d{a.x, a.y, a.z}, cov_a, dt);
        
    //     accel_callback_sim(a, cov_a);

    //     R3d translation = X.block<3, 1>(0, 4);
    //     SO3d rotation = Eigen::Quaterniond(X.block<3, 3>(0, 0));

    //     SE3d pose_in_map(translation, rotation);
    //     SE3Conversions::pushToTfTree(tf_broadcaster, ROVER_FRAME, MAP_FRAME, pose_in_map, get_clock()->now());

    //     last_imu_time = imu_msg.header.stamp;

    // }

    // void IEKF_SE3::pos_callback_sim(const geometry_msgs::msg::Vector3Stamped& pos_msg) {

    //     RCLCPP_INFO(get_logger(), "pos callback");

    //     Matrix39d H;
    //     Vector5d Y;
    //     Vector5d b;
    //     Matrix33d N;

    //     H << Matrix33d::Zero(), Matrix33d::Zero(), -1 * Matrix33d::Identity();
    //     Y << -1 * X.block<3, 3>(0, 0).transpose() * Vector3d{pos_msg.vector.x, pos_msg.vector.y, pos_msg.vector.z}, 0, 1;
    //     b << 0, 0, 0, 0, 1;
    //     N << X.block<3, 3>(0, 0) * 0.01 * Matrix33d::Identity() * X.block<3, 3>(0, 0).transpose();

    //     correct(Y, b, N, H);

    // }

    // void IEKF_SE3::mag_heading_callback_sim(const mrover::msg::Heading& mag_heading_msg) {

    //     Matrix39d H;
    //     Vector5d Y;
    //     Vector5d b;
    //     Matrix33d N;

    //     Vector3d mag_ref{1, 0, 0};

    //     double heading = mag_heading_msg.heading;

    //     // double heading = 90 - mag_heading_msg.heading;

    //     // if (heading < -180) {
    //     //     heading = 360 + heading;
    //     // }

    //     // heading = heading * (M_PI / 180);

    //     H << -1 * manif::skew(mag_ref), Matrix33d::Zero(), Matrix33d::Zero();
    //     Y << -1 * Vector3d{std::cos(heading), -std::sin(heading), 0}, 0, 0;
    //     b << -1 * mag_ref, 0, 0;
    //     N << X.block<3, 3>(0, 0) * 0.1 * X.block<3, 3>(0, 0).transpose();

    //     RCLCPP_INFO(get_logger(), "heading: %f", heading);
    //     // RCLCPP_INFO(get_logger(), "heading accuracy: %f", mag_heading_msg.heading_accuracy);
    //     correct(Y, b, N, H);
    
    // }

    // void IEKF_SE3::accel_callback_sim(const geometry_msgs::msg::Vector3 &a, const Matrix33d &cov_a) {

    //     Matrix39d H;
    //     Vector5d Y;
    //     Vector5d b;
    //     Matrix33d N;

    //     Vector3d accel_ref{0, 0, 1};

    //     Vector3d accel_meas{a.x, a.y, 9.81 + a.z};
    //     accel_meas.normalize();

    //     RCLCPP_INFO(get_logger(), "accel measured: %f, %f, %f", accel_meas(0), accel_meas(1), accel_meas(2));

    //     H << -1 * manif::skew(accel_ref), Matrix33d::Zero(), Matrix33d::Zero();
    //     Y << -1 * accel_meas, 0, 0;
    //     b << -1 * accel_ref, 0, 0;
    //     N << X.block<3, 3>(0, 0) * cov_a * X.block<3, 3>(0, 0).transpose();

    //     correct(Y, b, N, H);
    // }


}

int main(int argc, char**argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::IEKF_SE3>());
    rclcpp::shutdown();
    return 0;

}