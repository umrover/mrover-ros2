#include "iekf.hpp"
#include <builtin_interfaces/msg/detail/duration__struct.hpp>
#include <geometry_msgs/msg/detail/vector3_stamped__struct.hpp>

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
            
            geometry_msgs::msg::Vector3 w = imu_msg->angular_velocity;
            Matrix33d cov_w;
            cov_w << imu_msg->angular_velocity_covariance[0], imu_msg->angular_velocity_covariance[1], imu_msg->angular_velocity_covariance[2],
                     imu_msg->angular_velocity_covariance[3], imu_msg->angular_velocity_covariance[4], imu_msg->angular_velocity_covariance[5],
                     imu_msg->angular_velocity_covariance[6], imu_msg->angular_velocity_covariance[7], imu_msg->angular_velocity_covariance[8];
            
            double dt = GYRO_DT;
            if (last_imu_time) {
                dt = (imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9) - (last_imu_time.value().sec + last_imu_time.value().nanosec * 1e-9);   
            }
            gyro_callback(w, cov_w, dt);

            R3d position(0, 0, 0);
            SE3d pose_in_map(position, X.asSO3());
            SE3Conversions::pushToTfTree(tf_broadcaster, ROVER_FRAME, MAP_FRAME, pose_in_map, get_clock()->now());

            last_imu_time = imu_msg->header.stamp;
            
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
    

    auto IEKF::adjoint() -> Matrix99d {

        Matrix99d adj = Matrix99d::Zero();

        Matrix33d v_skew = manif::skew(X.translation());
        Matrix33d p_skew = manif::skew(X.linearVelocity());

        adj.block(0, 0, 3, 3) = X.rotation();
        adj.block(3, 0, 3, 3) = v_skew * X.rotation();
        adj.block(3, 3, 3, 3) = X.rotation();
        adj.block(6, 0, 3, 3) = p_skew * X.rotation();
        adj.block(6, 6, 3, 3) = X.rotation();

        return adj;

    }

    void IEKF::gyro_callback(const geometry_msgs::msg::Vector3& w, const Matrix33d& cov, double dt) {

        Matrix99d adj_x = adjoint();
        Matrix99d Q = Matrix99d::Zero();
        Matrix99d Q_d = Matrix99d::Zero();

        Q.block(0, 0, 3, 3) = cov.cwiseAbs();
        Q_d = (A * dt).exp() * Q * dt * ((A * dt).exp()).transpose();

        // propagate
        Vector9d u_vec;
        u_vec << Vector3d::Zero(), Vector3d{w.x, w.y, w.z} * dt, Vector3d::Zero();
        SE_2_3Tangentd u = u_vec;
        X = X + u;
        P = (A * dt).exp() * P * ((A * dt).exp()).transpose() + adj_x * Q_d * adj_x.transpose();

        // Matrix33d rotation_tracker = X.rotation();

        // RCLCPP_INFO(get_logger(), "%f, %f, %f\n %f, %f, %f\n %f, %f, %f", rotation_tracker(0,0), rotation_tracker(0,1), rotation_tracker(0,2),
        //                                                                     rotation_tracker(1,0), rotation_tracker(1,1), rotation_tracker(1,2),
        //                                                                     rotation_tracker(2,0), rotation_tracker(2,1), rotation_tracker(2,2));
    }

    // void IEKF::accel_callback(const geometry_msgs::msg::Vector3Stamped& a, const Matrix33d& cov, double dt) {

    //     Matrix99d adj_x = adjoint();
    //     Matrix99d Q = Matrix99d::Zero();
    //     Matrix99d Q_d = Matrix99d::Zero();

    //     Q.block(3, 3, 3, 3) = cov.cwiseAbs();
    //     Q.block(6, 6, 3, 3) = cov.cwiseAbs() * dt;
    //     Q_d = (A * dt).exp() * Q * dt * ((A * dt).exp()).transpose();

    //     Vector3d a_vec{a.vector.x, a.vector.y, a.vector.z};

    //     // propagate
    //     X.translation() = X.translation() + X.linearVelocity() * dt + 0.5 * X.rotation() * a_vec * pow(dt, 2) + 0.5 * g * pow(dt, 2);
    //     X.linearVelocity() = X.linearVelocity() + X.rotation() * a_vec * dt + g * dt;

    // }
    // auto IEKF::accelCallback(geometry_msgs::msg::Vector3Stamped accel) -> void {
        
    // }

    // auto magCallback() -> void {

    // }

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