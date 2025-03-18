#include "iekf.hpp"
#include <geometry_msgs/msg/detail/vector3_stamped__struct.hpp>
#include <rclcpp/parameter_value.hpp>

namespace mrover {
    
    IEKF::IEKF() : Node{"iekf"} {

        // declare ros params
        declare_parameter("world_frame", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("rover_frame", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("scale_cov_a", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("scale_cov_w", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("pos_noise_fixed", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("pos_noise_float", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("pos_noise_none", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("vel_noise_fixed", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("vel_noise_float", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("vel_noise_none", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("mag_heading_noise", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("rtk_heading_noise", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("near_zero_threshold", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("max_angular_vel_change_threshold", rclcpp::ParamaterType::PARAMETER_DOUBLE);
        declare_paramater("minimum_linear_speed", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("maximum_angular_speed", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("moving_window_sz", rclcpp::ParameterType::PARAMETER_INTEGER);
        declare_parameter("bufferSize", rclcpp::ParameterType::PARAMETER_INTEGER);

        scale_cov_a = get_parameter("scale_cov_a").as_double();
        scale_cov_w = get_parameter("scale_cov_w").as_double();
        pos_noise_fixed = get_parameter("pos_noise_fixed").as_double();
        vel_noise_fixed = get_parameter("vel_noise_fixed").as_double();
        mag_heading_noise = get_parameter("mag_heading_noise").as_double();
        max_angular_vel_change_threshold = get_parameter("max_angular_vel_change_threshold").as_double();
        minimum_linear_speed = get_parameter("minimum_linear_speed").as_double();
        maximum_angular_speed = get_parameter("maximum_angular_speed").as_double();
        near_zero_threshold = get_parameter("near_zero_threshold").as_double();
        moving_window_sz = get_parameter("moving_window_sz").as_int();
        bufferSize = get_parameter("bufferSize").as_int();

        // initialize state variables
        X.setIdentity();
        P = Matrix99d::Identity();
        A = Matrix99d::Zero();

        A.block(3, 0, 3, 3) = manif::skew(g);
        A.block(6, 3, 3, 3) = Matrix33d::Identity();

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
            vel_callback(vel_msg->vector);
        });

        //synchronizers 
        pos_and_imu_sync_ = std::make_shared<message_filters::Synchronizer<PosImuSyncPolicy>>(
            PosImuSyncPolicy(1), pos_sub, imu_sub);
        pos_and_imu_sync_->setAgePenalty(0.5);
        pose_and_imu_sync->registerCallback(&IEKF::drive_forward_callback, this);

        //watchdog timout
        imu_watchdog_timeout = this->create_wall_timer(IMU_WATCHDOG_TIMEOUT.to_chrono<std::chrono::milliseconds>(), [&]() {
            RCLCPP_WARN(get_logger(), "ZED IMU data timed out");
            last_imu_value.reset();
        })

        // subscribers sim
        // imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, [&](const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg) {
        //     imu_callback_sim(*imu_msg);
        // });

        // mag_heading_sub = this->create_subscription<mrover::msg::Heading>("/imu/mag", 10, [&](const mrover::msg::Heading::ConstSharedPtr& mag_heading_msg) {
        //     mag_heading_callback_sim(*mag_heading_msg);
        // });


    }


    auto IEKF::adjoint() -> Matrix99d {

        Matrix99d adj = Matrix99d::Zero();

        Matrix33d v_skew = manif::skew(X.col(3).head(3));
        Matrix33d p_skew = manif::skew(X.col(4).head(3));

        adj.block(0, 0, 3, 3) = X.block<3, 3>(0, 0);
        adj.block(3, 0, 3, 3) = v_skew * X.block<3, 3>(0, 0);
        adj.block(3, 3, 3, 3) = X.block<3, 3>(0, 0);
        adj.block(6, 0, 3, 3) = p_skew * X.block<3, 3>(0, 0);
        adj.block(6, 6, 3, 3) = X.block<3, 3>(0, 0);

        return adj;

    }

    auto lift(const Vector9d& dx) -> Matrix55d {
        
        Matrix55d result = Matrix55d::Zero();
        result.block<3, 3>(0, 0) = manif::skew(dx.head(3));
        result(0, 3) = dx(3);
        result(1, 3) = dx(4);
        result(2, 3) = dx(5);
        result(0, 4) = dx(6);
        result(1, 4) = dx(7);
        result(2, 4) = dx(8);

        return result;

    }


    void IEKF::predict(const Vector3d& w, const Matrix33d& cov_w, const Vector3d& a, const Matrix33d& cov_a, double dt) {

        Matrix99d adj_x = adjoint();
        Matrix99d Q = Matrix99d::Zero();
        Matrix99d Q_d = Matrix99d::Zero();

        // process noise
        Q.block(0, 0, 3, 3) = cov_w.cwiseAbs();
        Q.block(3, 3, 3, 3) = cov_a.cwiseAbs();
        Q.block(6, 6, 3, 3) = cov_a.cwiseAbs() * dt;
        Q_d = (A * dt).exp() * Q * dt * ((A * dt).exp()).transpose();

        // get linear acceleration in world frame
        Vector3d a_lin = X.block<3, 3>(0, 0) * a + g;

        // // simple bias estimator
        // if ((a_lin - accel_bias).norm() < BIAS_THRESHOLD) {
        //     accel_bias_estimator.push_back(a_lin);
        //     accel_bias = accel_bias + (a_lin - accel_bias) / accel_bias_estimator.size();

        //     if (accel_bias_estimator.size() > BIAS_WINDOW) {
        //         Vector3d old_value = accel_bias_estimator.front();
        //         accel_bias_estimator.pop_front();
        //         accel_bias = accel_bias + (accel_bias - old_value) / accel_bias_estimator.size();
        //     }

        // }

        // a_lin = a_lin - accel_bias;

        // discrimination window for accelerometer
        if (abs(a_lin(0)) < near_zero_threshold) {
            a_lin(0) = 0;
        }
        if (abs(a_lin(1)) < near_zero_threshold) {
            a_lin(1) = 0;
        }
        if (abs(a_lin(2)) < near_zero_threshold) {
            a_lin(2) = 0;
        }

        
        // propagate
        X.block<3, 3>(0, 0) = X.block<3, 3>(0, 0) * (manif::skew(w) * dt).exp();
        X.block<3, 1>(0, 4) = X.block<3, 1>(0, 4) + X.block<3, 1>(0, 3) * dt + 0.5 * a_lin * pow(dt, 2);
        X.block<3, 1>(0, 3) = X.block<3, 1>(0, 3) + a_lin * dt;
        
        P = (A * dt).exp() * P * ((A * dt).exp()).transpose() + adj_x * Q_d * adj_x.transpose();

        // Matrix33d rot_m = X.block<3, 3>(0, 0);
        // RCLCPP_INFO(get_logger(), "\n%f, %f, %f\n%f, %f, %f\n %f, %f, %f", rot_m(0,0), rot_m(0,1), rot_m(0,2),
        //                                                                          rot_m(1,0), rot_m(1,1), rot_m(1,2),
        //                                                                          rot_m(2,0), rot_m(2,1), rot_m(2,2));
    }


    void IEKF::correct(const Vector5d& Y, const Vector5d& b, const Matrix33d& N, const Matrix39d& H) {

        Vector5d innov = X * Y - b;
        Matrix33d S = H * P * H.transpose() + N;
        Matrix93d L = P * H.transpose() * S.inverse();
        Matrix55d dx = lift(L * innov.head(3));

        // RCLCPP_INFO(get_logger(), "innov: %f, %f, %f", innov(0), innov(1), innov(2));

        // Vector9d delta = L * innov.head(3);
        // Matrix55d exp_m = dx.exp();
        // RCLCPP_INFO(get_logger(), "delta: %f, %f, %f, %f, %f, %f, %f, %f, %f", delta(0), delta(1), delta(2), delta(3), delta(4), delta(5), delta(6), delta(7), delta(8));
        // RCLCPP_INFO(get_logger(), "L:\n%f, %f, %f\n%f, %f, %f\n %f, %f, %f\n%f, %f, %f\n%f, %f, %f\n%f, %f, %f\n%f, %f, %f\n%f, %f, %f\n%f, %f, %f", L(0,0), L(0,1), L(0,2),
        //                                                                          L(1,0), L(1,1), L(1,2),
        //                                                                          L(2,0), L(2,1), L(2,2),
        //                                                                          L(3,0), L(3,1), L(3,2),
        //                                                                          L(4,0), L(4,1), L(4,2),
        //                                                                          L(5,0), L(5,1), L(5,2),
        //                                                                          L(6,0), L(6,1), L(6,2),
        //                                                                          L(7,0), L(7,1), L(7,2),
        //                                                                          L(8,0), L(8,1), L(8,2));


        // RCLCPP_INFO(get_logger(), "expm:\n%f, %f, %f, %f, %f\n%f, %f, %f, %f, %f\n %f, %f, %f, %f, %f\n%f, %f, %f, %f, %f\n%f, %f, %f, %f, %f", exp_m(0,0), exp_m(0,1), exp_m(0,2), exp_m(0, 3), exp_m(0, 4),
        //                                                                          exp_m(1,0), exp_m(1,1), exp_m(1,2), exp_m(1, 3), exp_m(1, 4),
        //                                                                          exp_m(2,0), exp_m(2,1), exp_m(2,2), exp_m(2, 3), exp_m(2, 4),
        //                                                                          exp_m(3,0), exp_m(3,1), exp_m(3,2), exp_m(3, 3), exp_m(3, 4),
        //                                                                          exp_m(4,0), exp_m(4,1), exp_m(4,2), exp_m(4, 3), exp_m(4, 4));

        // RCLCPP_INFO(get_logger(), "X before:\n%f, %f, %f, %f, %f\n%f, %f, %f, %f, %f\n %f, %f, %f, %f, %f\n%f, %f, %f, %f, %f\n%f, %f, %f, %f, %f", X(0,0), X(0,1), X(0,2), X(0, 3), X(0, 4),
        //                                                                          X(1,0), X(1,1), X(1,2), X(1, 3), X(1, 4),
        //                                                                          X(2,0), X(2,1), X(2,2), X(2, 3), X(2, 4),
        //                                                                          X(3,0), X(3,1), X(3,2), X(3, 3), X(3, 4),
        //  
                                                    
        X = dx.exp() * X;
        P = (Matrix99d::Identity() - L * H) * P * (Matrix99d::Identity() - L * H).transpose() + L * N * L.transpose();
        
        RCLCPP_INFO(get_logger(), "X after:\n%f, %f, %f, %f, %f\n%f, %f, %f, %f, %f\n %f, %f, %f, %f, %f\n%f, %f, %f, %f, %f\n%f, %f, %f, %f, %f", X(0,0), X(0,1), X(0,2), X(0, 3), X(0, 4),
                                                                                 X(1,0), X(1,1), X(1,2), X(1, 3), X(1, 4),
                                                                                 X(2,0), X(2,1), X(2,2), X(2, 3), X(2, 4),
                                                                                 X(3,0), X(3,1), X(3,2), X(3, 3), X(3, 4),
                                                                                 X(4,0), X(4,1), X(4,2), X(4, 3), X(4, 4));
       

        
        
    }


    void IEKF::imu_callback(const sensor_msgs::msg::Imu& imu_msg) {

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



        Vector3d a_vec = Vector3d{a.x, a.y, a.z};
        moving_window.push_back(a_vec);


        accel_avg = accel_avg + (a_vec - accel_avg) / moving_window.size();

        if (moving_window.size() > moving_window_sz) {
            Vector3d old_value = moving_window.front();
            moving_window.pop_front();
            accel_avg = accel_avg + (accel_avg - old_value) / moving_window.size();
        }

        predict(Vector3d{w.x, w.y, w.z}, scale_cov_w * cov_w, accel_avg, scale_cov_a * cov_a, dt);
        
        accel_callback(a, scale_cov_a * cov_a);

        R3d translation = X.block<3, 1>(0, 4);
        SO3d rotation = Eigen::Quaterniond(X.block<3, 3>(0, 0));

        SE3d pose_in_map(translation, rotation);
        SE3Conversions::pushToTfTree(tf_broadcaster, get_parameter("rover_frame").as_string(), get_parameter("world_frame").as_string(), pose_in_map, get_clock()->now());

        last_imu_time = imu_msg.header.stamp;

    }

    void IEKF::pos_callback(const geometry_msgs::msg::Vector3& pos_msg) {

        RCLCPP_INFO(get_logger(), "pos callback");

        Matrix39d H;
        Vector5d Y;
        Vector5d b;
        Matrix33d N;

        H << Matrix33d::Zero(), Matrix33d::Zero(), -1 * Matrix33d::Identity();
        Y << -1 * X.block<3, 3>(0, 0).transpose() * Vector3d{pos_msg.x, pos_msg.y, pos_msg.z}, 0, 1;
        b << 0, 0, 0, 0, 1;
        N << X.block<3, 3>(0, 0) * pos_noise_fixed * Matrix33d::Identity() * X.block<3, 3>(0, 0).transpose();

        correct(Y, b, N, H);

        // geometry_msgs::msg::Vector3 vel_msg;
        // vel_msg.x = 0;
        // vel_msg.y = 0;
        // vel_msg.z = 0;
        // vel_callback(vel_msg);
        

    }

    void IEKF::vel_callback(const geometry_msgs::msg::Vector3& vel_msg) {

        RCLCPP_INFO(get_logger(), "vel callback");

        Matrix39d H;
        Vector5d Y;
        Vector5d b;
        Matrix33d N;

        H << Matrix33d::Zero(), -1 * Matrix33d::Identity(), Matrix33d::Zero();
        Y << -1 * X.block<3, 3>(0, 0).transpose() * Vector3d{vel_msg.x, vel_msg.y, vel_msg.z}, 1, 0;
        b << 0, 0, 0, 1, 0;
        N << X.block<3, 3>(0, 0) * vel_noise_fixed * Matrix33d::Identity() * X.block<3, 3>(0, 0).transpose();

        correct(Y, b, N, H);
    }

    

    void IEKF::mag_heading_callback(const mrover::msg::Heading& mag_heading_msg) {

        Matrix39d H;
        Vector5d Y;
        Vector5d b;
        Matrix33d N;

        Vector3d mag_ref{1, 0, 0};

        double heading = 90 - mag_heading_msg.heading;

        if (heading < -180) {
            heading = 360 + heading;
        }

        heading = heading * (M_PI / 180);

        H << -1 * manif::skew(mag_ref), Matrix33d::Zero(), Matrix33d::Zero();
        Y << -1 * Vector3d{std::cos(heading), -std::sin(heading), 0}, 0, 0;
        b << -1 * mag_ref, 0, 0;
        N << X.block<3, 3>(0, 0) * mag_heading_noise * Matrix33d::Identity() * X.block<3, 3>(0, 0).transpose();

        RCLCPP_INFO(get_logger(), "heading: %f", heading);
        correct(Y, b, N, H);
    
    }
    

    void IEKF::accel_callback(const geometry_msgs::msg::Vector3 &a, const Matrix33d &cov_a) {

        Matrix39d H;
        Vector5d Y;
        Vector5d b;
        Matrix33d N;

        Vector3d accel_ref{0, 0, 1};

        Vector3d accel_meas{a.x, a.y, a.z};
        accel_meas.normalize();

        RCLCPP_INFO(get_logger(), "accel measured: %f, %f, %f", accel_meas(0), accel_meas(1), accel_meas(2));

        H << -1 * manif::skew(accel_ref), Matrix33d::Zero(), Matrix33d::Zero();
        Y << -1 * accel_meas, 0, 0;
        b << -1 * accel_ref, 0, 0;
        N << X.block<3, 3>(0, 0) * cov_a * X.block<3, 3>(0, 0).transpose();

        correct(Y, b, N, H);
    }

    void IEKF::drive_forward_callback(const geometry_msgs::msg::Vector3& pos_msg, const sensor_msgs::msg::Imu& imu_msg) {
        
        if (pos_buffer.size() < bufferSize || imu_buffer.size() < bufferSize) {
            pos_buffer.push_back(Eigen::Vector3d(pos_msg.x, pos_msg.y, pos_msg.z));
            orientation_buffer.push_back(Eigen::Quaterniond(
                imu_msg.orientation.w, 
                imu_msg.orientation.x, 
                imu_msg.orientation.y, 
                imu_msg_orientation.z
            ));
            return;
        }

        Eigen::Vector3d old_position_in_map = pos_buffer.front();
        pos_buffer.pop_front();
        Eigen::Quaterniond old_orientation_in_map = orientation_buffer.front();
        orientation_buffer.pop_front();

        Eigen::Vector3d new_position_in_map(pos_msg.x, pos_msg.y, pos_msg.z);
        Eigen::Quaterniond new_orientation_in_map(imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z);
        pos_buffer.push_back(new_position_in_map);
        orientation_buffer.push_back(new_orientation_in_map);

        double deltaY = new_position_in_map.y() - old_position_in_map.y();
        double deltaX = new_position_in_map.x() - old_position_in_map.x();
        double angular_velocity = imu_msg.angular_velocity.z;

        double rover_vel_x = deltaX / STEP.seconds();
        double rover_vel_y = deltaY / STEP.seconds();
        double rover_velocity = sqrt(rover_vel_x * rover_vel_x + rover_vel_y * rover_vel_y);

        if (rover_velocity < minimum_linear_speed) {
            RCLCPP_WARN(this->get_logger(), "Rover stationary - insufficient speed: %.3f m/s", rover_velocity);
            return;
        } else {
            drive_forward_heading = atan2(deltaY, deltaX);
        }
        
        if (angular_velocity > maximum_angular_speed) {
            RCLCPP_WARN(this->get_logger(), "Rover changing heading too fast, angular velocity: %.3f m/s", angular_velocity);
            return;
        } else {
            drive_forward_heading = atan2(deltaY, deltaX);
        }
    }


    // void IEKF::predict_sim(const Vector3d& w, const Matrix33d& cov_w, const Vector3d& a, const Matrix33d& cov_a, double dt) {
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

    // void IEKF::imu_callback_sim(const sensor_msgs::msg::Imu& imu_msg) {

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

    // void IEKF::pos_callback_sim(const geometry_msgs::msg::Vector3Stamped& pos_msg) {

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

    // void IEKF::mag_heading_callback_sim(const mrover::msg::Heading& mag_heading_msg) {

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

    // void IEKF::accel_callback_sim(const geometry_msgs::msg::Vector3 &a, const Matrix33d &cov_a) {

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
    rclcpp::spin(std::make_shared<mrover::IEKF>());
    rclcpp::shutdown();
    return 0;

}