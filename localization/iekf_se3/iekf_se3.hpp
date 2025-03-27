#pragma once

// C++ Standard Library Headers, std namespace


// ROS Headers, ros namespace
#include "pch.hpp"

using Matrix33d = Eigen::Matrix<double, 3, 3>;
using Matrix44d = Eigen::Matrix<double, 5, 5>;
using Matrix66d = Eigen::Matrix<double, 6, 6>;
using Matrix36d = Eigen::Matrix<double, 3, 6>;
using Matrix63d = Eigen::Matrix<double, 6, 3>;
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Matrix<double, 5, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

namespace mrover {

    class IEKF_SE3 : public rclcpp::Node {

    private:

        // utils
        auto adjoint() -> Matrix66d;
        static auto lift(const Vector6d& dx) -> Matrix44d;

        // sensor callbacks
        void imu_callback(const sensor_msgs::msg::Imu& imu_msg);
        void pos_callback(const geometry_msgs::msg::Vector3& pos_msg);
        void mag_heading_callback(const mrover::msg::Heading& mag_heading_msg);
        void accel_callback(const geometry_msgs::msg::Vector3 &a, const Matrix33d &cov_a);
        void vel_callback(const geometry_msgs::msg::Vector3Stamped& vel_msg);

        // InEKF functions
        void predict_imu(const Vector3d& w, const Matrix33d& cov_w, double dt);
        void predict_vel(const Vector3d& v, const Matrix33d& cov_v, double dt);
        void correct(const Vector4d& Y, const Vector4d& b, const Matrix33d& N, const Matrix36d& H);

        // publishers and subscribers
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr pos_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<mrover::msg::Heading>::SharedPtr mag_heading_sub;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_sub;

        // tf broadcaster
        tf2_ros::Buffer tf_buffer{get_clock()};
        tf2_ros::TransformListener tf_listener{tf_buffer};
        tf2_ros::TransformBroadcaster tf_broadcaster{this};

        // timekeeping
        std::optional<builtin_interfaces::msg::Time> last_imu_time;
        std::optional<builtin_interfaces::msg::Time> last_vel_time;
       
        // state variables
        Matrix44d X;
        Matrix66d P;
        Matrix66d A;
        
        // constants
        const double IMU_DT = 0.016;
        const double VEL_DT = 1.0;
        const Vector3d g{0.0, 0.0, -9.81};

        // parameters
        std::string world_frame;
        std::string rover_frame;
        double scale_cov_a;
        double scale_cov_w;
        double pos_noise_fixed;
        double vel_noise;
        double mag_heading_noise;

    public:
    
        IEKF_SE3();
    }; // class IEKF

} // namespace mrover