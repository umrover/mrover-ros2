#include "iekf.hpp"

namespace mrover {
    
    IEKF::IEKF() : Node{"iekf"} {
        // TODO: pubs and subs and buffers(?)
    }

    auto IEKF::CorrectRightInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
                                const Eigen::MatrixXd& N) -> void {
        
        
        Eigen::VectorXd Theta = state.get_theta();
        Eigen::MatrixXd P = state.get_P();
        int dimX = state.dimX();
        int dimTheta = state.dimTheta();
        int dimP = state.dimP();

        // Remove bias
        // bool enable_imu_bias_update = state.get_enable_imu_bias_update();
        // if (!enable_imu_bias_update) {
        //     P.block<6, 6>(dimP - dimTheta, dimP - dimTheta)
        //         = 0.0001 * Eigen::Matrix<double, 6, 6>::Identity();
        //     P.block(0, dimP - dimTheta, dimP - dimTheta, dimTheta)
        //         = Eigen::MatrixXd::Zero(dimP - dimTheta, dimTheta);
        //     P.block(dimP - dimTheta, 0, dimTheta, dimP - dimTheta)
        //         = Eigen::MatrixXd::Zero(dimTheta, dimP - dimTheta);
        // }

        // // Map from left invariant to right invariant error temporarily
        // if (error_type == ErrorType::LeftInvariant) {
        //     Eigen::MatrixXd Adj = Eigen::MatrixXd::Identity(dimP, dimP);
        //     Adj.block(0, 0, dimP - dimTheta, dimP - dimTheta)
        //         = lie_group::Adjoint_SEK3(X);
        //     P = (Adj * P * Adj.transpose()).eval();
        // }

        // Compute Kalman Gain
        Eigen::MatrixXd PHT = P * H.transpose();
        Eigen::MatrixXd S = H * PHT + N;
        Eigen::MatrixXd K = PHT * S.inverse();
        // std::cout << "Kalman gain: \n" << K;
        // Compute state correction vector
        Eigen::VectorXd delta = K * Z;
        Eigen::MatrixXd dX
            = delta.segment(0, delta.rows() - dimTheta).exp();
        Eigen::VectorXd dTheta = delta.segment(delta.rows() - dimTheta, dimTheta);

        // Update state
        Eigen::MatrixXd X_new = dX * X;    // Right-Invariant Update
        /// REMARK: set yaw bias derivative estimation to 0
        dTheta(2) = 0;
        Eigen::VectorXd Theta_new = Theta + dTheta;

        // Set new state
        state.set_X(X_new);
        state.set_theta(Theta_new);

        // Update Covariance
        Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(dimP, dimP) - K * H;
        Eigen::MatrixXd P_new = IKH * P * IKH.transpose()
                                + K * N * K.transpose();    // Joseph update form

        // Don't update yaw covariance
        /// TODO: Add a flag to enable yaw covariance update
        P_new.row(dimP - dimTheta + 2).setZero();
        P_new.col(dimP - dimTheta + 2).setZero();
        P_new(dimP - dimTheta + 2, dimP - dimTheta + 2) = 0.0001 * 1;
        // Map from right invariant back to left invariant error
        if (error_type == ErrorType::LeftInvariant) {
            Eigen::MatrixXd AdjInv = Eigen::MatrixXd::Identity(dimP, dimP);
            AdjInv.block(0, 0, dimP - dimTheta, dimP - dimTheta)
                = lie_group::Adjoint_SEK3(state.get_Xinv());
            P_new = (AdjInv * P_new * AdjInv.transpose()).eval();
        }

        // Set new covariance
        state.set_P(P_new);
        // std::cout << "Covariance P: \n" << P_new;
        
    }

    auto IEKF::CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
                                const Eigen::MatrixXd& N) -> void {
        
    }

    Matrix99d IEKF::adjoint() {

        Matrix33d v_skew = Matrix33d::Zero();
        Matrix33d p_skew = Matrix33d::Zero();
        Matrix99d adj = Matrix99d::Zero();

        v_skew(0, 1) = -1 * X(3, 4);
        v_skew(0, 2) = X(2, 3);
        v_skew(1, 0) = X(3, 4);
        v_skew(1, 2) = -1 * X(1, 4);
        v_skew(2, 0) = -1 * X(2, 4);
        v_skew(2, 1) = X(1, 4);

        p_skew(0, 1) = -1 * X(3, 5);
        p_skew(0, 2) = X(2, 5);
        p_skew(1, 0) = X(3, 5);
        p_skew(1, 2) = -1 * X(1, 5);
        p_skew(2, 0) = -1 * X(2, 5);
        p_skew(2, 1) = X(1, 5);

        adj.block(0, 0, 3, 3) = X.block(0, 0, 3, 3);
        adj.block(3, 0, 3, 3) = v_skew * X.block(0, 0, 3, 3);
        adj.block(3, 3, 3, 3) = X.block(0, 0, 3, 3);
        adj.block(6, 0, 3, 3) = p_skew * X.block(0, 0, 3, 3);
        adj.block(6, 6, 3, 3) = obj.X(0, 0, 3, 3);

        return adj;

    }

    void IEKF::gyro_callback(geometry_msgs::msg::Vector3Stamped w, Matrix33d cov, double dt) {

        Matrix33d w_skew = Matrix33d::Zero();
        Matrix99d Q = Matrix99d::Zero();
        Matrix99d Q_d = Matrix99d::Zero();

        w_skew(0, 1) = -1 * w.vector.z;
        w_skew(0, 2) = w.vector.y;
        w_skew(1, 0) = w.vector.z;
        w_skew(1, 2) = -1 * w.vector.x;
        w_skew(2, 0) = -1 * w.vector.y;
        w_skew(2, 1) = w.vector.x;

        Matrix99d adj_x = adjoint();

        Q.block(0, 0, 3, 3) = cov;

        // Q_d = (A * dt).exp() * Q * dt * ((A * dt).exp()).transpose(); uh oh 

        // propagate
        X.block(0, 0, 3, 3) = X.block(0, 0, 3, 3) * (w_skew * dt).exp();
        P = (A * dt).exp() * P * ((A * dt).exp()).transpose() + adj_x * Q_d * adj_x.transpose();
        
    }

    auto IEKF::accelCallback(geometry_msgs::msg::Vector3Stamped accel) -> void {
        
    }

    auto magCallback() -> void {

    }

    auto gpsCallback() -> void {

    }

}