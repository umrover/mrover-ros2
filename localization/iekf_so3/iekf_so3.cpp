#include "iekf_so3.hpp"
#include <geometry_msgs/msg/detail/vector3__struct.hpp>

namespace mrover {
    
    IEKF_SO3::IEKF_SO3() : Node("iekf_so3") {

        // declare ros params
        declare_parameter("world_frame", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("rover_frame", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("scale_cov_a", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("scale_cov_w", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("mag_heading_noise", rclcpp::ParameterType::PARAMETER_DOUBLE);

        world_frame = get_parameter("world_frame").as_string();
        rover_frame = get_parameter("rover_frame").as_string();
        scale_cov_a = get_parameter("scale_cov_a").as_double();
        scale_cov_w = get_parameter("scale_cov_w").as_double();
        mag_heading_noise = get_parameter("mag_heading_noise").as_double();

        // initialize state variables
        X.setIdentity();
        P.setIdentity();
        A.setIdentity();

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

    }

    auto IEKF_SO3::adjoint() -> Matrix33d { return X; }

    auto IEKF_SO3::lift(const Vector3d& dx) -> Matrix33d {
        Matrix33d result = manif::skew(dx);
        return result;
    }

    void IEKF_SO3::predict(const Vector3d& w, const Matrix33d& cov_w, double dt) {

        Matrix33d adj_x = adjoint();
        Matrix33d Q = Matrix33d::Zero();
        Matrix33d Q_d = Matrix33d::Zero();

        // covariance
        Q = cov_w.cwiseAbs();
        Q_d = (A * dt).exp() * Q * dt * ((A * dt).exp()).transpose();

        // propagate
        X = X * (manif::skew(w) * dt).exp();
        P = (A * dt).exp() * P * ((A * dt).exp()).transpose() + adj_x * Q_d * adj_x.transpose();
        
    }

    void IEKF_SO3::correct(const Vector3d& Y, const Vector3d& b, const Matrix33d& N, const Matrix33d& H) {

        Vector3d innov = X * Y - b;
        Matrix33d S = H * P * H.transpose() + N;
        Matrix33d L = P * H.transpose() * S.inverse();
        Matrix33d dx = lift(L * innov.head(3));
                                          
        X = dx.exp() * X;
        P = (Matrix33d::Identity() - L * H) * P * (Matrix33d::Identity() - L * H).transpose() + L * N * L.transpose();
       
    }

    void IEKF_SO3::accel_callback(const geometry_msgs::msg::Vector3& a, const Matrix33d& cov_a) {

        Matrix33d H;
        Vector3d Y;
        Vector3d b;
        Matrix33d N;

        Vector3d accel_ref{0, 0, 1};

        Vector3d accel_meas{a.x, a.y, a.z};
        accel_meas.normalize();

        H << -1 * manif::skew(accel_ref), Matrix33d::Zero();
        Y << -1 * accel_meas, 0;
        b << -1 * accel_ref, 0;
        N << X * cov_a * X.transpose();

        correct(Y, b, N, H);
    }

    void IEKF_SO3::imu_callback(const sensor_msgs::msg::Imu& imu_msg) {
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

        predict(Vector3d{w.x, w.y, w.z}, scale_cov_w * cov_w, dt);

        accel_callback(a, cov_a);

        if (last_position) {

            R3d translation = last_position.value();
            SO3d rotation = Eigen::Quaterniond(X);

            SE3d pose_in_map(translation, rotation);
            SE3Conversions::pushToTfTree(tf_broadcaster, rover_frame, world_frame, pose_in_map, get_clock()->now());

        }
        else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No position data, cannott publish to TF");
        }

        last_imu_time = imu_msg.header.stamp;
        
    }

    void IEKF_SO3::mag_heading_callback(const mrover::msg::Heading& mag_heading_msg) {
        Matrix33d H;
        Vector3d Y;
        Vector3d b;
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
        N << X * mag_heading_noise * Matrix33d::Identity() * X.transpose();

        correct(Y, b, N, H);
    }

    void IEKF_SO3::pos_callback(const geometry_msgs::msg::Vector3& pos_msg) {
        last_position = {pos_msg.x, pos_msg.y, pos_msg.z};
    }


}

int main(int argc, char**argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::IEKF_SO3>());
    rclcpp::shutdown();
    return 0;

}