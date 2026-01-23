#include "pch.hpp"

namespace mrover {
    
    class PoseFilter : public rclcpp::Node {
    
    private:

        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr imu_sub;
        message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped> pos_sub;
        message_filters::Subscriber<mrover::msg::FixStatus> pos_status_sub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::ConstSharedPtr cmd_vel_sub;

        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime
            <geometry_msgs::msg::Vector3Stamped, mrover::msg::FixStatus>>> pos_sync;

        void imu_callback(const sensor_msgs::msg::Imu &imu);
        void pos_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &pos_msg, const mrover::msg::FixStatus::ConstSharedPtr &pos_status_msg);
        void drive_forward_callback();

        // data store
        std::optional<SO3d> curr_heading_correction;
        std::optional<sensor_msgs::msg::Imu> last_imu;
        std::optional<geometry_msgs::msg::Vector3> last_pos;
        std::vector<geometry_msgs::msg::Twist> twists;

        // transform utilities
        tf2_ros::Buffer tf_buffer{get_clock()};
        tf2_ros::TransformListener tf_listener{tf_buffer};
        tf2_ros::TransformBroadcaster tf_broadcaster{this};

        // timekeeping
        rclcpp::TimerBase::SharedPtr imu_watchdog;
        rclcpp::TimerBase::SharedPtr correction_timer;

        // constants
        const rclcpp::Duration IMU_WATCHDOG_TIMEOUT = rclcpp::Duration::from_seconds(1.0);
        const rclcpp::Duration STEP = rclcpp::Duration::from_seconds(0.5);
        const rclcpp::Duration WINDOW{STEP * 2.5};

        // parameters
        std::string world_frame;
        std::string gps_frame;
        double rover_heading_change_threshold;
        double minimum_linear_speed;

    public:
        PoseFilter();
        
        
    };
}
