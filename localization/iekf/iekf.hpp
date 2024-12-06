#pragma once

// C++ Standard Library Headers, std namespace


// ROS Headers, ros namespace
#include "pch.hpp"

using manif::SO3d;
using Matrix33d = Eigen::Matrix<double, 3, 3>;
using Matrix55d = Eigen::Matrix<double, 5, 5>;
using Matrix99d = Eigen::Matrix<double, 9, 9>;

namespace mrover {

    class IEKF : public rclcpp::Node {

        private:

            // IEKF variables
            Matrix55d X;
            Matrix99d P;
            Matrix99d A;

            rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr pos_sub;
            rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr gyro_sub;
            rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_sub;
            rclcpp::Subscription<mrover::msg::Heading>::SharedPtr heading_sub;
            
            void gyro_callback(geometry_msgs::msg::Vector3Stamped w, Matrix33d cov, double dt);
            Matrix99d adjoint();


        protected:
            

            auto CorrectRightInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
                                    const Eigen::MatrixXd& N) -> void; // ErrorType error_type

            auto CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
                            const Eigen::MatrixXd& N) -> void; // ErrorType error_type



            auto gyroCallback(geometry_msgs::msg::Vector3Stamped vel) -> void;
            auto accelCallback(geometry_msgs::msg::Vector3Stamped accel) -> void;

            auto magCallback() -> void;

            auto gpsCallback() -> void;
        public:
            IEFK();
    };

} // namespace mrover