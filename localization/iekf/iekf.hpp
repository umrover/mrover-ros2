#pragma once

// C++ Standard Library Headers, std namespace


// ROS Headers, ros namespace
#include "pch.hpp"

using SO3d = manif::SO3d;
using SE_2_3d = manif::SE_2_3d;
using SE_2_3Tangentd = manif::SE_2_3Tangentd;
using Matrix33d = Eigen::Matrix<double, 3, 3>;
using Matrix55d = Eigen::Matrix<double, 5, 5>;
using Matrix99d = Eigen::Matrix<double, 9, 9>;
using Matrix39d = Eigen::Matrix<double, 3, 9>;
using Vector3d = Eigen::Vector3d;
using Vector5d = Eigen::Matrix<double, 5, 1>;
using Vector9d = Eigen::Matrix<double, 9, 1>;

namespace mrover {

    class IEKF : public rclcpp::Node {

    private:

        // utils
        auto adjoint() -> Matrix99d;

        // sensor callbacks
        void imu_callback(const sensor_msgs::msg::Imu& imu_msg);
        void pos_callback(const geometry_msgs::msg::Vector3Stamped& pos_msg);

        // InEKF functions
        void predict(const geometry_msgs::msg::Vector3& w, const Matrix33d& cov_w, const geometry_msgs::msg::Vector3& a, const Matrix33d& cov_a, double dt);
        void correct(const Vector5d& Y, const Vector5d& b, const Matrix33d& n, const Matrix39d& H);

        // publishers and subscribers
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr pos_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<mrover::msg::Heading>::SharedPtr rtk_heading_sub;
        rclcpp::Subscription<mrover::msg::Heading>::SharedPtr mag_heading_sub;

        // tf broadcaster
        tf2_ros::Buffer tf_buffer{get_clock()};
        tf2_ros::TransformListener tf_listener{tf_buffer};
        tf2_ros::TransformBroadcaster tf_broadcaster{this};

        // timekeeping
        std::optional<builtin_interfaces::msg::Time> last_imu_time;
       
        // state variables
        SE_2_3d X;
        Matrix99d P;
        Matrix99d A;
        Vector3d g{0, 0, -9.80665};
        
        const double IMU_DT = 0.016;
        const std::string ROVER_FRAME = "base_link";
        const std::string MAP_FRAME = "map";

        
        

        // auto CorrectRightInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
        //                         const Eigen::MatrixXd& N) -> void; // ErrorType error_type

        // auto CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
        //                 const Eigen::MatrixXd& N) -> void; // ErrorType error_type



        // auto gyroCallback(geometry_msgs::msg::Vector3Stamped vel) -> void;
        // auto accelCallback(geometry_msgs::msg::Vector3Stamped accel) -> void;

        // auto magCallback() -> void;

        auto gpsCallback(geometry_msgs::msg::Vector3Stamped position, geometry_msgs::msg::Vector3Stamped V) -> void;

    public:
    
        IEKF();
    }; // class IEKF

} // namespace mrover