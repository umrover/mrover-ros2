#pragma once

#include "mrover/msg/detail/heading__struct.hpp"
#include "pch.hpp"

using Quaterniond = Eigen::Quaterniond;
using Matrix33d = Eigen::Matrix<double, 3, 3>;
using Matrix66d = Eigen::Matrix<double, 6, 6>;
using Matrix36d = Eigen::Matrix<double, 3, 6>;
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Matrix<double, 4, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;


namespace mrover {

    class QuatIEKF : public rclcpp::Node {

    private:

        // utils
        auto static quat_to_vec(const Quaterniond& q) -> Vector4d;
        auto static lift_q(const Vector3d& v) -> Quaterniond;

        // Quat IEKF functions
        void predict(const Vector3d& w, const Vector3d& n_v, const Vector3d& n_u, double dt);
        void correct(const Matrix36d& H, const Matrix33d& N, const Vector3d& observation, const Vector3d& predicted);

        // sensor callbacks
        void imu_callback(const sensor_msgs::msg::Imu& imu_msg);
        void mag_heading_callback(const mrover::msg::Heading& mag_heading_msg);

        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

        // tf broadcaster
        tf2_ros::Buffer tf_buffer{get_clock()};
        tf2_ros::TransformListener tf_listener{tf_buffer};
        tf2_ros::TransformBroadcaster tf_broadcaster{this};

        // timekeeping
        std::optional<builtin_interfaces::msg::Time> last_imu_time;

        const double IMU_DT = 0.016;
        const std::string ROVER_FRAME = "base_link";
        const std::string MAP_FRAME = "map";


        Quaterniond q;
        Vector3d B;
        Quaterniond q_err;
        Vector3d B_err;
        Matrix66d P;


    public:
        QuatIEKF();



    };

};

