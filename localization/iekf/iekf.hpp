#pragma once

// C++ Standard Library Headers, std namespace


// ROS Headers, ros namespace
#include "pch.hpp"


namespace mrover {

    class IEKF : public rclcpp::Node {

        private:

            // IEKF variables
            Eigen::Matrix<double, 5, 5> X;
            Eigen::Matrix<double, 9, 9> P;
            Eigen::Matrix<double, 9, 9> A;

            rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr pos_sub;
            rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr gyro_sub;
            rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_sub;
            rclcpp::Subscription<mrover::msg::Heading>::SharedPtr heading_sub;
            



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