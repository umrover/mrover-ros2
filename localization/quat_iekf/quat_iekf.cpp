#include "quat_iekf.hpp"

namespace mrover {

    QuatIEKF::QuatIEKF() : Node("quat_iekf") {

        q.setIdentity();
        B.setZero();
        q_err.setIdentity();
        B_err.setZero();
        P.setIdentity();

        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, [&](const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg) {
            imu_callback(*imu_msg);
        });

        mag_heading_sub = this->create_subscription<mrover::msg::Heading>("/imu/mag", 10, [&](const mrover::msg::Heading::ConstSharedPtr& mag_heading_msg) {
            mag_heading_callback(*mag_heading_msg);
        });


    }

    auto QuatIEKF::quat_to_vec(const Quaterniond& q) -> Vector4d {
        return Vector4d{q.w(), q.x(), q.y(), q.z()};
    }

    auto QuatIEKF::lift_q(const Vector3d& v) -> Quaterniond {

        double scalar_comp = std::cos(v.norm() / 2);
        Vector3d vector_comp = v.normalized() * std::sin(v.norm() / 2);
        return Quaterniond{scalar_comp, vector_comp(0), vector_comp(1), vector_comp(2)};

    }


    void QuatIEKF::predict(const Vector3d& w, const Vector3d& n_v, const Vector3d& n_u, double dt) {

        Vector3d w_meas = w - B;

        Quaterniond n_v_est = q * Quaterniond{0, n_v(0), n_v(1), n_v(2)} * q.conjugate();
        Quaterniond I_w_est = q * Quaterniond{0, w_meas(0), w_meas(1), w_meas(2)} * q.conjugate();
        Quaterniond n_u_est = q * Quaterniond{0, n_u(0), n_u(1), n_u(2)} * q.conjugate();

        // propagate error terms
        Vector4d q_err_vec = quat_to_vec(q_err);
        
        q_err_vec = q_err_vec + quat_to_vec(Quaterniond(-0.5 * q_err_vec) * Quaterniond(0, B_err.x(), B_err.y(), B_err.z())) * dt + 
                                quat_to_vec(Quaterniond(0.5 * quat_to_vec(n_v_est)) * q_err) * dt;

        Vector3d B_err_intermediary = q_err.toRotationMatrix() * (I_w_est.vec() - n_u_est.vec());

        B_err = B_err + B_err_intermediary.cross(B_err) * dt - q_err.toRotationMatrix() * n_u_est.vec() * dt;

        q_err = Quaterniond(q_err_vec);
        q_err.normalize();

        Matrix66d F;
        Matrix66d G;
        Matrix66d Q;
        Vector6d N_vec;

        F << Matrix33d::Zero(), -Matrix33d::Identity(), Matrix33d::Zero(), manif::skew(I_w_est.vec());
        G << q.toRotationMatrix().transpose(), Matrix33d::Zero(), Matrix33d::Zero(), -1 * (q.toRotationMatrix().transpose());
        N_vec << n_v, n_u;
        Q = N_vec.asDiagonal();

        Matrix66d F_d = (F * dt).exp();

        // propagate state, error covariance matrix
        P = F_d * P * F_d.transpose() + G * F_d * Q * dt * F_d.transpose() * G.transpose();
        q = lift_q(w_meas * dt) * q; // see https://math.stackexchange.com/questions/39553/how-do-i-apply-an-angular-velocity-vector3-to-a-unit-quaternion-orientation
        q.normalize();

        RCLCPP_INFO(get_logger(), "Quat: %f, %f, %f, %f", q.x(), q.y(), q.z(), q.w());
        Matrix33d rot_m = q.toRotationMatrix();
        RCLCPP_INFO(get_logger(), "rot_m:\n%f, %f, %f\n%f, %f, %f\n %f, %f, %f", rot_m(0,0), rot_m(0,1), rot_m(0,2),
                                                                                 rot_m(1,0), rot_m(1,1), rot_m(1,2),
                                                                                 rot_m(2,0), rot_m(2,1), rot_m(2,2));

    }

    void QuatIEKF::correct(const Matrix36d& H, const Matrix33d& R, const Vector3d& observation, const Vector3d& predicted) {

        Vector3d innovation = predicted - observation;
        Matrix63d K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        Vector6d correction = K * innovation;

        RCLCPP_INFO(get_logger(), "innov: %f, %f, %f", innovation(0), innovation(1), innovation(2));
        RCLCPP_INFO(get_logger(), "correction: %f, %f, %f", correction(0), correction(1), correction(2));

        q = lift_q(Vector3d{-correction(0), -correction(1), -correction(2)}) * q;
        q.normalize();
        Quaterniond B_update = q.conjugate() * Quaterniond{0, correction(3), correction(4), correction(5)} * q;
        B = B - Vector3d{B_update.x(), B_update.y(), B_update.z()};
        P = (Matrix66d::Identity() - K * H) * P;

    }

    void QuatIEKF::imu_callback(const sensor_msgs::msg::Imu& imu_msg) {

        Vector3d w{imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z};
        Vector3d n_v{imu_msg.angular_velocity_covariance[0], imu_msg.angular_velocity_covariance[4], imu_msg.angular_velocity_covariance[8]};
        Vector3d n_u{0, 0, 0};

        geometry_msgs::msg::Vector3 a = imu_msg.linear_acceleration;
        Matrix33d cov_a;
        cov_a << imu_msg.linear_acceleration_covariance[0], imu_msg.linear_acceleration_covariance[1], imu_msg.linear_acceleration_covariance[2],
                 imu_msg.linear_acceleration_covariance[3], imu_msg.linear_acceleration_covariance[4], imu_msg.linear_acceleration_covariance[5],
                 imu_msg.linear_acceleration_covariance[6], imu_msg.linear_acceleration_covariance[7], imu_msg.linear_acceleration_covariance[8];
        
        double dt = IMU_DT;
        if (last_imu_time) {
            dt = (imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9) - (last_imu_time.value().sec + last_imu_time.value().nanosec * 1e-9);   
        }

        predict(w, n_v, n_u, dt);

        accel_callback(a, cov_a);

        last_imu_time = imu_msg.header.stamp;

    }

    void QuatIEKF::mag_heading_callback(const mrover::msg::Heading& mag_heading_msg) {

        Matrix36d H;
        Matrix33d R;
        Vector3d observation;
        Vector3d predicted;

        double heading = mag_heading_msg.heading;
        Vector3d mag_ref{1, 0, 0};
        Quaterniond observation_q = q * Quaterniond{0, std::cos(heading), -std::sin(heading), 0} * q.conjugate();
        // Quaterniond observation_q = q * Quaterniond{0, std::cos(heading), std::sin(heading), 0} * q.conjugate();
        RCLCPP_INFO(get_logger(), "mag observation: %f, %f, %f", observation_q.x(), observation_q.y(), observation_q.z());

        H << manif::skew(mag_ref), Matrix33d::Zero();
        R << q.toRotationMatrix().transpose() * mag_heading_msg.heading_accuracy * Matrix33d::Identity() * q.toRotationMatrix();
        observation << observation_q.x(), observation_q.y(), observation_q.z();
        predicted << mag_ref;

        RCLCPP_INFO(get_logger(), "heading: %f", heading);
        correct(H, R, observation, predicted);

    }

    void QuatIEKF::accel_callback(const geometry_msgs::msg::Vector3& accel_msg, const Matrix33d& cov_a) {

        Matrix36d H;
        Matrix33d R;
        Vector3d observation;
        Vector3d predicted;

        Vector3d accel_meas{accel_msg.x, accel_msg.y, -9.81 + accel_msg.z};
        accel_meas.normalize();

        Vector3d accel_ref{0, 0, -1};
        Quaterniond observation_q = q * Quaterniond{0, accel_meas(0), accel_meas(1), accel_meas(2)} * q.conjugate();
        RCLCPP_INFO(get_logger(), "accel observation: %f, %f, %f", observation_q.x(), observation_q.y(), observation_q.z());

        H << manif::skew(accel_ref), Matrix33d::Zero();
        R << q.toRotationMatrix().transpose() * cov_a * q.toRotationMatrix();
        observation << observation_q.x(), observation_q.y(), observation_q.z();
        predicted << accel_ref;

        correct(H, R, observation, predicted);
        
        


    }



};

int main(int argc, char**argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::QuatIEKF>());
    rclcpp::shutdown();
    return 0;

}