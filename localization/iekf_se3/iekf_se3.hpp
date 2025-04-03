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
        void pos_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& pos_msg, const mrover::msg::FixStatus::ConstSharedPtr& pos_status_msg);
        void mag_heading_callback(const mrover::msg::Heading& mag_heading_msg);
        void accel_callback(const geometry_msgs::msg::Vector3 &a, const Matrix33d &cov_a);
        void vel_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& vel_msg, const mrover::msg::FixStatus::ConstSharedPtr& vel_status_msg);
        void drive_forward_callback();
        void rtk_heading_callback(const mrover::msg::Heading::ConstSharedPtr& rtk_heading, const mrover::msg::FixStatus::ConstSharedPtr& rtk_heading_status);

        // InEKF functions
        void predict_imu(const Vector3d& w, const Matrix33d& cov_w, double dt);
        void predict_vel(const Vector3d& v, const Matrix33d& cov_v, double dt);
        void correct(const Vector4d& Y, const Vector4d& b, const Matrix33d& N, const Matrix36d& H);

        // publishers and subscribers
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<mrover::msg::Heading>::SharedPtr mag_heading_sub;

        message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped> pos_sub;
        message_filters::Subscriber<mrover::msg::FixStatus> pos_status_sub;
        message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped> velocity_sub;
        message_filters::Subscriber<mrover::msg::FixStatus> velocity_status_sub;
        message_filters::Subscriber<mrover::msg::Heading> rtk_heading_sub;
        message_filters::Subscriber<mrover::msg::FixStatus> rtk_heading_status_sub;

        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime
            <geometry_msgs::msg::Vector3Stamped, mrover::msg::FixStatus>>> pos_sync;

        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime
            <geometry_msgs::msg::Vector3Stamped, mrover::msg::FixStatus>>> velocity_sync;

        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime
            <mrover::msg::Heading, mrover::msg::FixStatus>>> rtk_heading_sync;

        // tf broadcaster
        tf2_ros::Buffer tf_buffer{get_clock()};
        tf2_ros::TransformListener tf_listener{tf_buffer};
        tf2_ros::TransformBroadcaster tf_broadcaster{this};

        // timekeeping
        std::optional<builtin_interfaces::msg::Time> last_imu_time;
        std::optional<builtin_interfaces::msg::Time> last_vel_time;
        rclcpp::TimerBase::SharedPtr correction_timer;
       
        // state variables
        Matrix44d X;
        Matrix66d P;
        Matrix66d A;
        
        // constants
        const double IMU_DT = 0.016;
        const double VEL_DT = 1.0;
        const Vector3d g{0.0, 0.0, -9.81};
        const rclcpp::Duration STEP = rclcpp::Duration::from_seconds(0.5);
        const rclcpp::Duration WINDOW{STEP * 2.5};

        // parameters
        std::string world_frame;
        std::string rover_frame;
        double scale_cov_a;
        double scale_cov_w;
        double pos_noise_fixed;
        double vel_noise;
        double mag_heading_noise;
        double rtk_heading_noise;

        double rover_heading_change_threshold;
        double minimum_linear_speed;

        bool use_mag;
        bool use_drive_forward;

    public:
    
        IEKF_SE3();
    }; // class IEKF

} // namespace mrover