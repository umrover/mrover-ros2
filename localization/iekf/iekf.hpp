#pragma once

// C++ Standard Library Headers, std namespace


// ROS Headers, ros namespace
#include "pch.hpp"

using SO3d = manif::SO3d;
using SE_2_3d = manif::SE_2_3d;
using Matrix33d = Eigen::Matrix<double, 3, 3>;
using Matrix55d = Eigen::Matrix<double, 5, 5>;
using Matrix99d = Eigen::Matrix<double, 9, 9>;
using Matrix39d = Eigen::Matrix<double, 3, 9>;
using Vector3d = Eigen::Vector3d;

// using Vector5d = Eigen::Matrix<double, 4, 1>;

namespace mrover {

    class IEKF : public rclcpp::Node {

    private:

        // utils
        auto adjoint() -> Matrix99d;
        static auto sup_x(const Vector3d& v) -> Matrix33d;

        // sensor callbacks
        void gyro_callback(const geometry_msgs::msg::Vector3Stamped& w, const Matrix33d& cov, double dt);
        void accel_callback(const geometry_msgs::msg::Vector3Stamped& a, const Matrix33d& cov, double dt);

        // publishers and subscribers
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr pos_sub;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr gyro_sub;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_sub;
        rclcpp::Subscription<mrover::msg::Heading>::SharedPtr rtk_heading_sub;
        rclcpp::Subscription<mrover::msg::Heading>::SharedPtr mag_heading_sub;

        // state variables
        SE_2_3d X;
        Matrix99d P;
        Matrix99d A;
        Vector3d g{0, 0, 9.80665};
        
        const double dt = 0.01;


        
        

        // auto CorrectRightInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
        //                         const Eigen::MatrixXd& N) -> void; // ErrorType error_type

        // auto CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
        //                 const Eigen::MatrixXd& N) -> void; // ErrorType error_type



        // auto gyroCallback(geometry_msgs::msg::Vector3Stamped vel) -> void;
        // auto accelCallback(geometry_msgs::msg::Vector3Stamped accel) -> void;

        // auto magCallback() -> void;

        // auto gpsCallback(geometry_msgs::msg::Vector3Stamped position, geometry_msgs::msg::Vector3Stamped V) -> void;

    public:
    
        IEKF();
    }; // class IEKF

} // namespace mrover