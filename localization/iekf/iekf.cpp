#include "iekf.hpp"
#include <geometry_msgs/msg/detail/vector3_stamped__struct.hpp>

namespace mrover {
    
    IEKF::IEKF() : Node{"iekf"} {
        // TODO: pubs and subs and buffers(?)


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

    auto IEKF::sup_x(const Vector3d& v) -> Matrix33d {
        Matrix33d result = Matrix33d::Zero();

        result(0, 1) = -v(2);
        result(0, 2) = v(1);
        result(1, 0) = v(2);
        result(1, 2) = -v(0);
        result(2, 0) = -v(1);
        result(2, 1) = v(0);

        return result;
    }

    

    auto IEKF::adjoint() -> Matrix99d {

        Matrix99d adj = Matrix99d::Zero();

        Matrix33d v_skew = sup_x(X.translation());
        Matrix33d p_skew = sup_x(X.linearVelocity());


        adj.block(0, 0, 3, 3) = X.rotation();
        adj.block(3, 0, 3, 3) = v_skew * X.rotation();
        adj.block(3, 3, 3, 3) = X.rotation();
        adj.block(6, 0, 3, 3) = p_skew * X.rotation();
        adj.block(6, 6, 3, 3) = X.rotation();

        return adj;

    }

    void IEKF::gyro_callback(const geometry_msgs::msg::Vector3Stamped& w, const Matrix33d& cov, double dt) {

        Matrix33d w_skew = sup_x(Vector3d{w.vector.x, w.vector.y, w.vector.z});
        Matrix99d adj_x = adjoint();
        Matrix99d Q = Matrix99d::Zero();
        Matrix99d Q_d = Matrix99d::Zero();

        Q.block(0, 0, 3, 3) = cov.cwiseAbs();
        Q_d = (A * dt).exp() * Q * dt * ((A * dt).exp()).transpose();

        // propagate
        X.rotation() = X.rotation() * (w_skew * dt).exp();
        P = (A * dt).exp() * P * ((A * dt).exp()).transpose() + adj_x * Q_d * adj_x.transpose();
    }

    void IEKF::accel_callback(const geometry_msgs::msg::Vector3Stamped& a, const Matrix33d& cov, double dt) {

        Matrix99d adj_x = adjoint();
        Matrix99d Q = Matrix99d::Zero();
        Matrix99d Q_d = Matrix99d::Zero();

        Q.block(3, 3, 3, 3) = cov.cwiseAbs();
        Q.block(6, 6, 3, 3) = cov.cwiseAbs() * dt;
        Q_d = (A * dt).exp() * Q * dt * ((A * dt).exp()).transpose();

        Vector3d a_vec{a.vector.x, a.vector.y, a.vector.z};

        // propagate
        X.translation() = X.translation() + X.linearVelocity() * dt + 0.5 * X.rotation() * a * pow(dt, 2) + 0.5 * g * pow(dt, 2);
        X.linearVelocity() = X.linearVelocity() + X.rotation() * a_vec * dt + g * dt;

    }
    // auto IEKF::accelCallback(geometry_msgs::msg::Vector3Stamped accel) -> void {
        
    // }

    // auto magCallback() -> void {

    // }

    // auto gpsCallback(geometry_msgs::msg::Vector3Stamped position, geometry_msgs::msg::Vector3Stamped V) -> void {
    //     Matrix39d H = Matrix39d::Zero();
    //     H[0,6] = -1;
    //     H[1,7] = -1;
    //     H[2,8] = -1;
        
    //     Vector5d observation = X.inverse().act(position.vector) + V.vector;

    //     Matrix33d N = Matrix33d::Zero();
    //     N[0][0] = V.vector.x * V.vector.x
    //     N[1][1] = V.vector.y * V.vector.y
    //     N[2][2] = V.vector.z * V.vector.z

    //     Matrix33d E = H * obj.P * H.transpose();
    //     Matrix33d S = E + N;
    //     Matrix33d L = P * H.transpose() * S.inverse() // Kalman Gain

    //     Eigen::Vector3d innov = X.act(observation) - position
        
    //     // Correction factor (delta)
    //     Eigen::Vector3d dx = L * innovation;

    //     // Update 
    //     X = dx + X; // Overloaded lplus -- exp(dx) * X
    //     Matrix99d cov_factor = Matrix99d::Identity() - L*H;
    //     P = cov_factor * P * cov_factor.transpose() + L * N * L.tranpose();
    // }

}

int main(int argc, char**argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::IEKF>());
    rclcpp::shutdown();
    return 0;

}