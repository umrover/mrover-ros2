#include "iekf.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>

namespace mrover {
    
    IEKF::IEKF() : Node{"iekf"} {

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

    }


    // I think this was copied from drift, all naming conventions and such are cooked
    // auto IEKF::CorrectRightInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
    //                             const Eigen::MatrixXd& N) -> void {
        
        
    //     Eigen::VectorXd Theta = state.get_theta();
    //     Eigen::MatrixXd P = state.get_P();
    //     int dimX = state.dimX();
    //     int dimTheta = state.dimTheta();
    //     int dimP = state.dimP();

    //     // Remove bias
    //     // bool enable_imu_bias_update = state.get_enable_imu_bias_update();
    //     // if (!enable_imu_bias_update) {
    //     //     P.block<6, 6>(dimP - dimTheta, dimP - dimTheta)
    //     //         = 0.0001 * Eigen::Matrix<double, 6, 6>::Identity();
    //     //     P.block(0, dimP - dimTheta, dimP - dimTheta, dimTheta)
    //     //         = Eigen::MatrixXd::Zero(dimP - dimTheta, dimTheta);
    //     //     P.block(dimP - dimTheta, 0, dimTheta, dimP - dimTheta)
    //     //         = Eigen::MatrixXd::Zero(dimTheta, dimP - dimTheta);
    //     // }

    //     // // Map from left invariant to right invariant error temporarily
    //     // if (error_type == ErrorType::LeftInvariant) {
    //     //     Eigen::MatrixXd Adj = Eigen::MatrixXd::Identity(dimP, dimP);
    //     //     Adj.block(0, 0, dimP - dimTheta, dimP - dimTheta)
    //     //         = lie_group::Adjoint_SEK3(X);
    //     //     P = (Adj * P * Adj.transpose()).eval();
    //     // }

    //     // Compute Kalman Gain
    //     Eigen::MatrixXd PHT = P * H.transpose();
    //     Eigen::MatrixXd S = H * PHT + N;
    //     Eigen::MatrixXd L = PHT * S.inverse();
    //     // std::cout << "Kalman gain: \n" << K;
    //     // Compute state correction vector
    //     Eigen::VectorXd delta = L * Z;
    //     Eigen::MatrixXd dX
    //         = delta.segment(0, delta.rows() - dimTheta).exp();
    //     Eigen::VectorXd dTheta = delta.segment(delta.rows() - dimTheta, dimTheta);

    //     // Update state
    //     Eigen::MatrixXd X_new = dX * X;    // Right-Invariant Update
    //     /// REMARK: set yaw bias derivative estimation to 0
    //     dTheta(2) = 0;
    //     Eigen::VectorXd Theta_new = Theta + dTheta;

    //     // Set new state
    //     state.set_X(X_new);
    //     state.set_theta(Theta_new);

    //     // Update Covariance
    //     Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(dimP, dimP) - L * H;
    //     Eigen::MatrixXd P_new = IKH * P * IKH.transpose()
    //                             + L * N * L.transpose(); 

    //     // Don't update yaw covariance
    //     /// TODO: Add a flag to enable yaw covariance update
    //     P_new.row(dimP - dimTheta + 2).setZero();
    //     P_new.col(dimP - dimTheta + 2).setZero();
    //     P_new(dimP - dimTheta + 2, dimP - dimTheta + 2) = 0.0001 * 1;
    //     // Map from right invariant back to left invariant error
    //     if (error_type == ErrorType::LeftInvariant) {
    //         Eigen::MatrixXd AdjInv = Eigen::MatrixXd::Identity(dimP, dimP);
    //         AdjInv.block(0, 0, dimP - dimTheta, dimP - dimTheta)
    //             = lie_group::Adjoint_SEK3(state.get_Xinv());
    //         P_new = (AdjInv * P_new * AdjInv.transpose()).eval();
    //     }

    //     // Set new covariance
    //     state.set_P(P_new);
    //     // std::cout << "Covariance P: \n" << P_new;
        
    // }

    // auto IEKF::CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
    //                             const Eigen::MatrixXd& N) -> void {
        
    // }
    

    // auto IEKF::adjoint() -> Matrix99d {

    //     Matrix99d adj = Matrix99d::Zero();

    //     Matrix33d v_skew = manif::skew(X.linearVelocity());
    //     Matrix33d p_skew = manif::skew(X.translation());

    //     adj.block(0, 0, 3, 3) = X.rotation();
    //     adj.block(3, 0, 3, 3) = v_skew * X.rotation();
    //     adj.block(3, 3, 3, 3) = X.rotation();
    //     adj.block(6, 0, 3, 3) = p_skew * X.rotation();
    //     adj.block(6, 6, 3, 3) = X.rotation();

    //     return adj;

    // }

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

    // void IEKF::predict(const geometry_msgs::msg::Vector3& w, const Matrix33d& cov_w, const geometry_msgs::msg::Vector3& a, const Matrix33d& cov_a, double dt) {

    //     Matrix99d Q = Matrix99d::Zero();
    //     Matrix99d Q_d = Matrix99d::Zero();

    //     // process noise
    //     Q.block(0, 0, 3, 3) = cov_w.cwiseAbs();
    //     Q.block(3, 3, 3, 3) = cov_a.cwiseAbs();
    //     Q.block(6, 6, 3, 3) = cov_a.cwiseAbs() * dt;
    //     Q_d = (A * dt).exp() * Q * dt * ((A * dt).exp()).transpose();

    //     // get linear acceleration in world frame
    //     Vector3d a_lin = X.rotation() * Vector3d{a.x, a.y, a.z} + g;

    //     // simple bias estimator
    //     if ((a_lin - accel_bias).norm() < BIAS_THRESHOLD) {
    //         accel_bias_estimator.push_back(a_lin);
    //         accel_bias = accel_bias + (a_lin - accel_bias) / accel_bias_estimator.size();

    //         if (accel_bias_estimator.size() > BIAS_WINDOW) {
    //             Vector3d old_value = accel_bias_estimator.front();
    //             accel_bias_estimator.pop_front();
    //             accel_bias = accel_bias + (accel_bias - old_value) / accel_bias_estimator.size();
    //         }

    //     }

    //     a_lin = a_lin - accel_bias;

    //     // propagate
    //     Vector9d u_vec;
    //     u_vec << (X.linearVelocity() * dt + 0.5 * a_lin * pow(dt, 2)), Vector3d{w.x, w.y, w.z} * dt, a_lin * dt;
    //     SE_2_3Tangentd u = u_vec;
    //     X = X + u;
    //     P = (A * dt).exp() * P * ((A * dt).exp()).transpose() + X.adj() * Q_d * X.adj().transpose();

       
    //     // RCLCPP_INFO(get_logger(), "bias: %f, %f, %f", accel_bias(0), accel_bias(1), accel_bias(2));
    //     // RCLCPP_INFO(get_logger(), "a_lin: %f, %f, %f", a_lin(0), a_lin(1), a_lin(2));
    //     position_temp = position_temp + velocity_temp * dt + 0.5 * a_lin * pow(dt, 2);
    //     velocity_temp = velocity_temp + a_lin * dt;
        
    //     // RCLCPP_INFO(get_logger(), "velocity temp: %f, %f, %f", velocity_temp(0), velocity_temp(1), velocity_temp(2));
    //     RCLCPP_INFO(get_logger(), "position temp: %f, %f, %f", position_temp(0), position_temp(1), position_temp(2));
    //     // RCLCPP_INFO(get_logger(), "accel measurement: %f, %f, %f", a.x, a.y, a.z);
    //     // RCLCPP_INFO(get_logger(), "\n%f, %f, %f\n%f, %f, %f\n %f, %f, %f", rot_m(0,0), rot_m(0,1), rot_m(0,2),
    //     //                                                                          rot_m(1,0), rot_m(1,1), rot_m(1,2),
    //     //                                                                          rot_m(2,0), rot_m(2,1), rot_m(2,2));

                                                                    
    // }

    void IEKF::predict(const geometry_msgs::msg::Vector3& w, const Matrix33d& cov_w, const geometry_msgs::msg::Vector3& a, const Matrix33d& cov_a, double dt) {

        Matrix99d adj_x = adjoint();
        Matrix99d Q = Matrix99d::Zero();
        Matrix99d Q_d = Matrix99d::Zero();

        // process noise
        Q.block(0, 0, 3, 3) = cov_w.cwiseAbs();
        Q.block(3, 3, 3, 3) = cov_a.cwiseAbs();
        Q.block(6, 6, 3, 3) = cov_a.cwiseAbs() * dt;
        Q_d = (A * dt).exp() * Q * dt * ((A * dt).exp()).transpose();

        // get linear acceleration in world frame
        Vector3d a_lin = X.block<3, 3>(0, 0) * Vector3d{a.x, a.y, a.z} + g;

        // simple bias estimator
        if ((a_lin - accel_bias).norm() < BIAS_THRESHOLD) {
            accel_bias_estimator.push_back(a_lin);
            accel_bias = accel_bias + (a_lin - accel_bias) / accel_bias_estimator.size();

            if (accel_bias_estimator.size() > BIAS_WINDOW) {
                Vector3d old_value = accel_bias_estimator.front();
                accel_bias_estimator.pop_front();
                accel_bias = accel_bias + (accel_bias - old_value) / accel_bias_estimator.size();
            }

        }

        a_lin = a_lin - accel_bias;

        // propagate
        X.block<3, 3>(0, 0) = X.block<3, 3>(0, 0) * (manif::skew(Vector3d{w.x, w.y, w.z}) * dt).exp();
        X.block<3, 1>(0, 4) = X.block<3, 1>(0, 4) + X.block<3, 1>(0, 3) * dt + 0.5 * a_lin * pow(dt, 2);
        X.block<3, 1>(0, 3) = X.block<3, 1>(0, 3) + a_lin * dt;
        
        P = (A * dt).exp() * P * ((A * dt).exp()).transpose() + adj_x * Q_d * adj_x.transpose();

        Matrix33d rot_m = X.block<3, 3>(0, 0);
        RCLCPP_INFO(get_logger(), "\n%f, %f, %f\n%f, %f, %f\n %f, %f, %f", rot_m(0,0), rot_m(0,1), rot_m(0,2),
                                                                                 rot_m(1,0), rot_m(1,1), rot_m(1,2),
                                                                                 rot_m(2,0), rot_m(2,1), rot_m(2,2));

       
    //     // RCLCPP_INFO(get_logger(), "bias: %f, %f, %f", accel_bias(0), accel_bias(1), accel_bias(2));
    //     // RCLCPP_INFO(get_logger(), "a_lin: %f, %f, %f", a_lin(0), a_lin(1), a_lin(2));
    //     position_temp = position_temp + velocity_temp * dt + 0.5 * a_lin * pow(dt, 2);
    //     velocity_temp = velocity_temp + a_lin * dt;
        
    //     // RCLCPP_INFO(get_logger(), "velocity temp: %f, %f, %f", velocity_temp(0), velocity_temp(1), velocity_temp(2));
    //     RCLCPP_INFO(get_logger(), "position temp: %f, %f, %f", position_temp(0), position_temp(1), position_temp(2));
    //     // RCLCPP_INFO(get_logger(), "accel measurement: %f, %f, %f", a.x, a.y, a.z);
    //     // RCLCPP_INFO(get_logger(), "\n%f, %f, %f\n%f, %f, %f\n %f, %f, %f", rot_m(0,0), rot_m(0,1), rot_m(0,2),
    //     //                                                                          rot_m(1,0), rot_m(1,1), rot_m(1,2),
    //     //                                                                          rot_m(2,0), rot_m(2,1), rot_m(2,2));
    }

    // void IEKF::correct(const Vector3d& Y, const Vector3d& b, const Matrix33d& N, const Matrix39d& H) {
        
    //     Vector3d innov = X.act(Y) - b;
    //     // Vector3d temp = X.act(Y);
    //     // RCLCPP_INFO(get_logger(), "temp: %f, %f, %f", temp(0), temp(1), temp(2));
    //     Matrix33d S = H * P * H.transpose() + N;
    //     Matrix93d L = P * H.transpose() * S.inverse();
    //     Vector9d delta = L * innov;
    //     RCLCPP_INFO(get_logger(), "delta:, %f, %f, %f, %f, %f, %f, %f, %f, %f", delta(0), delta(1), delta(2), delta(3), delta(4), delta(5), delta(6), delta(7), delta(8));
    //     Vector9d dx_vec;
    //     dx_vec << delta(6, 9), delta(0, 3), delta(3, 6);
    //     SE_2_3Tangentd dx = dx_vec;

    //     X = dx + X;
    //     Vector3d pos_after = X.translation();
    //     RCLCPP_INFO(get_logger(), "corrected: %f, %f, %f", pos_after(0), pos_after(1), pos_after(2));
    //     P = (Matrix99d::Identity() - L * H) * P * (Matrix99d::Identity() - L * H).transpose() + L * N * L.transpose();

    void IEKF::correct(const Vector5d& Y, const Vector5d& b, const Matrix33d& N, const Matrix39d& H) {

        // RCLCPP_INFO(get_logger(), "Y: %f, %f, %f", Y(0), Y(1), Y(2));
        Vector5d innov = X * Y - b;
        Vector5d temp = X * Y;
        RCLCPP_INFO(get_logger(), "X * Y: %f, %f, %f, %f, %f", temp(0), temp(1), temp(2), temp(3), temp(4));
        RCLCPP_INFO(get_logger(), "innov: %f, %f, %f", innov(0), innov(1), innov(2));
        Matrix33d S = H * P * H.transpose() + N;
        Matrix93d L = P * H.transpose() * S.inverse();
        Matrix55d dx = lift(L * innov.head(3));

        Vector9d delta = L * innov.head(3);
        RCLCPP_INFO(get_logger(), "delta: %f, %f, %f, %f, %f, %f, %f, %f, %f", delta(0), delta(1), delta(2), delta(3), delta(4), delta(5), delta(6), delta(7), delta(8));
        X = dx.exp() * X;
        P = (Matrix99d::Identity() - L * H) * P * (Matrix99d::Identity() - L * H).transpose() + L * N * L.transpose();
        
        Matrix33d rot_m = X.block<3, 3>(0, 0);
        RCLCPP_INFO(get_logger(), "\n%f, %f, %f\n%f, %f, %f\n %f, %f, %f", rot_m(0,0), rot_m(0,1), rot_m(0,2),
                                                                                 rot_m(1,0), rot_m(1,1), rot_m(1,2),
                                                                                 rot_m(2,0), rot_m(2,1), rot_m(2,2));
        // Vector3d translation_temp = X.block<3, 1>(0, 4);
        // RCLCPP_INFO(get_logger(), "translation temp: %f, %f, %f", translation_temp(0), translation_temp(1), translation_temp(2));
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

        predict(w, cov_w, a, cov_a, dt);

        R3d translation = X.block<3, 1>(0, 4);
        SO3d rotation = Eigen::Quaterniond(X.block<3, 3>(0, 0));

        // R3d translation = X.translation();
        // SO3d rotation = X.asSO3();

        SE3d pose_in_map(translation, rotation);
        SE3Conversions::pushToTfTree(tf_broadcaster, ROVER_FRAME, MAP_FRAME, pose_in_map, get_clock()->now());

        last_imu_time = imu_msg.header.stamp;

        fake_gps++;

        if (fake_gps == 50) {
            geometry_msgs::msg::Vector3Stamped msg;
            msg.header.stamp = now();
            msg.vector.x = 1;
            msg.vector.y = 0;
            msg.vector.z = 0;
            pos_callback(msg);
            fake_gps = 0;
        }

    }

    // void IEKF::pos_callback(const geometry_msgs::msg::Vector3Stamped& pos_msg) {

    //     Matrix39d H;
    //     Vector3d Y = -1 * X.rotation().transpose() * Vector3d{pos_msg.vector.x, pos_msg.vector.y, pos_msg.vector.z};
    //     Vector3d b = Vector3d::Zero();
    //     Matrix33d N;

    //     H << Matrix33d::Zero(), Matrix33d::Zero(), -1 * Matrix33d::Identity();
    //     N << X.rotation() * 0.01 * Matrix33d::Identity() * X.rotation().transpose();

    //     correct(Y, b, N, H);

    // }

    void IEKF::pos_callback(const geometry_msgs::msg::Vector3Stamped& pos_msg) {

        Matrix39d H;
        Vector5d Y;
        Vector5d b;
        Matrix33d N;

        H << Matrix33d::Zero(), Matrix33d::Zero(), -1 * Matrix33d::Identity();
        Y << -1 * X.block<3, 3>(0, 0).transpose() * Vector3d{pos_msg.vector.x, pos_msg.vector.y, pos_msg.vector.z}, 0, 1;
        b << 0, 0, 0, 0, 1;
        N << X.block<3, 3>(0, 0) * 0.01 * Matrix33d::Identity() * X.block<3, 3>(0, 0).transpose();

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
        N << X.block<3, 3>(0, 0) * mag_heading_msg.heading_accuracy * Matrix33d::Identity() * X.block<3, 3>(0, 0).transpose();

        RCLCPP_INFO(get_logger(), "heading: %f", heading);
        // RCLCPP_INFO(get_logger(), "heading accuracy: %f", mag_heading_msg.heading_accuracy);
        correct(Y, b, N, H);

    }

    

    
    // auto IEKF::gpsCallback(geometry_msgs::msg::Vector3Stamped position, geometry_msgs::msg::Vector3Stamped V) -> void {
    //     Matrix39d H = Matrix39d::Zero();
    //     H(0,6) = -1;
    //     H(1,7) = -1;
    //     H(2,8) = -1;
        
    //     Vector5d position_affine;
    //     position_affine << position.vector.x,  position.vector.y, position.vector.z, 0, 1; 
    //     Vector5d V_affine;
    //     V_affine << V.vector.x,  V.vector.y, V.vector.z, 0, 1; 
    //     Vector5d observation = X.inverse().act(position_affine) + V_affine;

    //     Matrix33d N = Matrix33d::Zero();
    //     N(0,0) = V.vector.x * V.vector.x;
    //     N(1,1) = V.vector.y * V.vector.y;
    //     N(2,2)= V.vector.z * V.vector.z;

    //     Matrix33d E = H * P * H.transpose();
    //     Matrix33d S = E + N;
    //     Matrix33d L = P * H.transpose() * S.inverse(); // Kalman Gain

    //     Vector5d innov = X.act(observation) - position_affine;
        
    //     // Correction factor (delta)
    //     Vector5d dx = L * innov;

    //     // Update 
    //     X = dx + X; // Overloaded lplus -- exp(dx) * X
    //     Matrix99d cov_factor = Matrix99d::Identity() - L*H;
    //     P = cov_factor * P * cov_factor.transpose() + L * N * L.transpose();
    // }

}

int main(int argc, char**argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::IEKF>());
    rclcpp::shutdown();
    return 0;

}