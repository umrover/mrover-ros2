#pragma once

#include "pch.hpp"

using Matrix33d = Eigen::Matrix<double, 3, 3>;
using Vector3d = Eigen::Vector3<double>;

namespace mrover {
    class IEKF_SO3 : public rclcpp::Node {

    private:
        // utils
        auto adjoint() -> Matrix33d;
        static auto lift(const Vector3d& dx) -> Matrix33d;

        // sensor callbacks
        void imu_callback(const sensor_msgs::msg::Imu& imu_msg);
        void accel_callback(const geometry_msgs::msg::Vector3& a, const Matrix33d& cov_a);
        void mag_heading_callback(const mrover::msg::Heading& mag_heading_msg);
        void pos_callback(const geometry_msgs::msg::Vector3& pos_msg);

        // InEKF functions
        void predict(const Vector3d& w, const Matrix33d& cov_w, double dt);
        void correct(const Vector3d& Y, const Vector3d& b, const Matrix33d& N, const Matrix33d& H);

        // publishers and subscribers
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr pos_sub;
        rclcpp::Subscription<mrover::msg::Heading>::SharedPtr mag_heading_sub;

        // tf broadcaster
        tf2_ros::Buffer tf_buffer{get_clock()};
        tf2_ros::TransformListener tf_listener{tf_buffer};
        tf2_ros::TransformBroadcaster tf_broadcaster{this};

        // data store
        std::optional<Vector3d> last_position;
        std::optional<builtin_interfaces::msg::Time> last_imu_time;

        // state variables
        Matrix33d X;
        Matrix33d P;
        Matrix33d A;

        // constants
        const double IMU_DT = 0.016;

        // parameters
        std::string world_frame;
        std::string rover_frame;
        double scale_cov_a;
        double scale_cov_w;
        double mag_heading_noise;

    public:
        IEKF_SO3();

    };

} // namespace mrover