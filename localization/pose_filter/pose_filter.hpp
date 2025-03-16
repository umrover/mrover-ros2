#pragma once

#include "pch.hpp"

namespace mrover {

    class PoseFilter : public rclcpp::Node {

    private:

        auto ros_quat_to_eigen_quat(geometry_msgs::msg::Quaternion const& q) -> Eigen::Quaterniond;

        //Pose callbacks & filtering
        void pose_sub_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &linearized_pos_msg);
        void rtk_heading_callback(const mrover::msg::Heading::ConstSharedPtr &heading_msg, const mrover::msg::FixStatus::ConstSharedPtr &fix_status_msg);
        void imu_and_mag_callback(const mrover::msg::Imu::ConstSharedPtr &imu, const mrover::msg::Heading::ConstSharedPtr &mag_heading);
        void compare_and_select_heading();
        void correction_timer_callback();

        //Pose subscribers and publishers
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr linearized_position_sub_;
        message_filters::Subscriber<mrover::msg::Heading> rtk_heading_sub_;
        message_filters::Subscriber<mrover::msg::FixStatus> rtk_fix_status_sub_;
        message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
        message_filters::Subscriber<mrover::msg::Heading> mag_heading_sub_;

        //Pose synchronizers 
        using RTKSyncPolicy = message_filters::sync_policies::ApproximateTime<
            mrover::msg::Heading, 
            mrover::msg::FixStatus>;
        std::shared_ptr<message_filters::Synchronizer<RTKSyncPolicy>> rtk_sync_;
        
        using ImuMagSyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Imu, 
            mrover::msg::Heading>;
        std::shared_ptr<message_filters::Synchronizer<ImuMagSyncPolicy>> imu_and_mag_sync_;

        //transform utilities 
        tf2ros::Buffer tf_buffer{get_clock()};
        tf2ros::TransformListener tf_listener{tf_buffer};
        tf2ros::TransformBroadcaster tf_broadcaster{this};

        //ZED IMU data watchdog
        rclcpp::TimerBase::SharedPtr imu_watchdog_timeout;
        rclcpp::TimerBase::SharedPtr correction_timer;

        //Pose data store
        std::optional<geometry_msgs::msg::Vector3Stamped> last_position;
        std::optional<mrover::msg::Heading> last_rtk_heading;
        std::optional<mrover::msg::FixStatus> last_rtk_fix_status;
        std::optional<sensor_msgs::msg::Imu> last_imu;
        std::optional<mrover::msg::Heading> last_mag_heading;
        std::optional<SO3d> curr_heading_correction_
        std::optional<double> averaged_pose_and_correction_heading;
        std::optional<double> pose_callback_heading;
        std::optional<double> correction_timer_heading;
        std::optional<double> pose_heading_correction_delta; 
        std::optional<double> correction_timer_heading_delta; 

        //Heading data thresholding
        const rclcpp::Duration IMU_WATCHDOG_TIMEOUT = rclcpp::Duration::from_seconds(1.0);
        constexpr static float HEADING_THRESHOLD = (M_PI / 16);

        //Correction timer subscribers and publishers 
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
        rclcpp::Subscription<geometry::msg::Twist>::SharedPtr twist_sub_;

        std::string rover_frame;
        std::string world_frame;

        const rclcpp::Duration STEP = rclcpp::Duration::from_seconds(0.5);
        const rclcpp::Duration IMU_WATCHDOG_TIMEOUT = rclcpp::Duration::from_seconds(1.0);
        const rclcpp::Duration WINDOW{STEP * 2.5};
        
        static constexpr std::uint8_t FULL_CALIBRATION = 3;
        static constexpr float MIN_LINEAR_SPEED = 0.2, MAX_ANGULAR_SPEED = 0.1, MAX_ANGULAR_CHANGE = 0.2;
        
        rclcpp::Subscription<mrover::msg::CalibrationStatus>::SharedPtr calibration_status_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_uncalib_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_calib_sub;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr pose_sub;

        std::optional<mrover::msg::CalibrationStatus> calibration_status;
        std::vector<geometry_msgs::msg::Twist> twists;
        std::optional<sensor_msgs::msg::Imu> current_imu_uncalib;
        std::optional<sensor_msgs::msg::Imu> current_imu_calib;

        std::optional<SE3d> last_pose_in_map;
        std::optional<builtin_interfaces::msg::Time> last_pose_time;
        std::optional<SO3d> correction_rotation;

    public:
        PoseFilter();
    };

}
