#include "pose_filter.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

namespace mrover {
    

    auto PoseFilter::ros_quat_to_eigen_quat(geometry_msgs::msg::Quaternion const& q) -> Eigen::Quaterniond {
        return {q.w, q.x, q.y, q.z};
    }

    PoseFilter::PoseFilter() : Node("pose_filter") {
        
        correction_timer = this->create_wall_timer(WINDOW.to_chrono<std::chrono::milliseconds>(), [this]() -> void {
            correction_timer_callback();
        });

        odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 1);

        twist_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, [&](geometry_msgs::msg::Twist::ConstSharedPtr const& twist) {
            twists.push_back(*twist);
        });

        zed_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/zed_imu/data_raw", 1, [&](sensor_msgs::msg::Imu::ConstSharedPtr const& imu) {
            current_imu = *imu;
        });

        heading_sub_ = std::make_shared<message_filters::Subscriber<mrover::msg::Heading>>(this, "/heading/fix");

        heading_fix_sub_ = std::make_shared<message_filters::Subscriber<mrover::msg::FixStatus>>(this, "/heading_fix_status");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), *heading_sub_, *heading_fix_sub_
        );

        sync_->registerCallback(std::bind(&PoseFilter::heading_callback(), this, std::placeholders::_1, std::placeholders::_2));

        pose_sub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("/linearized_position", 1, [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr const& msg) -> void {
            pose_sub_callback(linearized_pos_msg);
        });

        declare_parameter("rover_frame", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("world_frame", rclcpp::ParameterType::PARAMETER_STRING);

        rover_frame = get_parameter("rover_frame").as_string();
        world_frame = get_parameter("world_frame").as_string();

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

    void PoseFilter::heading_callback(const mrover::msg::Heading::ConstSharedPtr const& heading_msg, const mrover::msg::FixStatus::ConstSharedPtr const& fix_status_msg) {
        current_heading *= heading_msg;
        current_heading_fix_status *= fix_status_msg;
    }

    void PoseFilter::pose_sub_callback(geometry_msgs::msg::Vector3Stamped::ConstSharedPtr const& linearized_pos_msg) {
\       if (!current_imu.has_value() || !current_heading.has_value() || !current_heading_fix_status.has_value()) {
            RCLPP_WARN(get_logger(), "Missing IMU, heading or heading fix status data. Ignoring pose correction via DA-RTK Heading");
            return;
        }

        if ((current_heading_fix_status->fix_status.fix != mrover::msg::FixType::FIXED)) {
            RCLPP_WARN(get_logger(), "Invalid heading fix status. Ignoring pose correction via DA-RTK Heading");
            return;
        }

        Eigen::Vector3d position_in_map(linearized_pos_msg->vector.x, linearized_pos_msg->vector.y, linearized_pos_msg->vector.z);

        auto const& imu_msg = current_imu.value();
        Eigen::Quaterniond quaternion(imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z);
        quaternion.normalize();

        double heading_rad = heading_msg->heading * (M_PI / 180);

        Eigen::Quaterniond desired_quaternion(Eigen::AngleAxisd(heading_rad, Eigen::Vector3d::UnitZ())); //you want to keep roll and pitch the same. 
        Eigen::Quaterniond transformation_quaternion = quaternion * desired_quaternion.inverse();
        Eigen::Quaterniond corrected_quaternion = transformation_quaternion * quaternion;
        corrected_quaternion.normalize();

        SE3d pose_in_map(position_in_map, corrected_quaternion);

        SE3Conversion::pushToTfTree(tf_broadcaster, rover_frame, world_frame, pose_in_map, getClock()->now())

        RCLCPP_INFO(get_logger(), "Published to TF Tree: Position (%f, %f, %f), Orientation (%f, %f, %f, %f)",
            position_in_map.x(), position_in_map.y(), position_in_map.z(),
            corrected_quaternion.x(), corrected_quaternion.y(). corrected_quaternion.z(), corrected_quaternion.w()
        );

    }


    // void PoseFilter::pose_sub_callback(geometry_msgs::msg::Vector3Stamped::ConstSharedPtr const& msg) {

    //     R3d position_in_map(msg->vector.x, msg->vector.y, msg->vector.z);

    //     SE3d pose_in_map(position_in_map, SO3d::Identity());

    //     bool mag_fully_calibrated = calibration_status && calibration_status->magnetometer_calibration == FULL_CALIBRATION;

    //     if (!mag_fully_calibrated && current_imu_calib && correction_rotation) {
    //         SO3d uncalibrated_orientation = ros_quat_to_eigen_quat(current_imu_uncalib->orientation);
    //         pose_in_map.asSO3() = correction_rotation.value() * uncalibrated_orientation;
    //     }
    //     else if (current_imu_calib) {
    //         SO3d calibrated_orientation = ros_quat_to_eigen_quat(current_imu_calib->orientation);
    //         pose_in_map.asSO3() = calibrated_orientation;
    //     }
    //     else {
    //         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1, "Not enough IMU data available");
    //         return;
    //     }

    //     SE3Conversions::pushToTfTree(tf_broadcaster, rover_frame, world_frame, pose_in_map, get_clock()->now());


        SE3d::Tangent twist;
        if (last_pose_in_map && last_pose_time) {
            twist = (pose_in_map - last_pose_in_map.value()) / ((msg->header.stamp.sec + msg->header.stamp.nanosec * 10e-9) - (last_pose_time->sec + last_pose_time->nanosec * 10e-9));
        }

        nav_msgs::msg::Odometry odometry;
        odometry.header = msg->header;
        odometry.pose.pose.position.x = pose_in_map.translation().x();
        odometry.pose.pose.position.y = pose_in_map.translation().y();
        odometry.pose.pose.position.z = pose_in_map.translation().z();
        odometry.pose.pose.orientation.w = pose_in_map.quat().w();
        odometry.pose.pose.orientation.x = pose_in_map.quat().x();
        odometry.pose.pose.orientation.y = pose_in_map.quat().y();
        odometry.pose.pose.orientation.z = pose_in_map.quat().z();
        odometry.twist.twist.linear.x = twist.coeffs()(0);
        odometry.twist.twist.linear.y = twist.coeffs()(1);
        odometry.twist.twist.linear.z = twist.coeffs()(2);
        odometry.twist.twist.angular.x = twist.coeffs()(3);
        odometry.twist.twist.angular.y = twist.coeffs()(4);
        odometry.twist.twist.angular.z = twist.coeffs()(5);
        odometry_pub->publish(odometry);

        last_pose_in_map = pose_in_map;
        last_pose_time = msg->header.stamp;

    }

    void PoseFilter::correction_timer_callback() {

        // 1. Ensure the rover is being commanded to move relatively straight forward

        geometry_msgs::msg::Twist mean_twist;

        for (auto total = static_cast<double>(twists.size()); auto const& twist : twists) {
            mean_twist.linear.x += twist.linear.x / total;
            mean_twist.angular.z += twist.angular.z / total;
        }

        twists.clear();

        if (!current_imu.has_value()) return;
        if (mean_twist.linear.x < MIN_LINEAR_SPEED) return;
        if (std::fabs(mean_twist.angular.z) > MAX_ANGULAR_SPEED) return;

        RCLCPP_INFO(get_logger(), "Rover is being commanded forward");

        // Compute the past velocities and headings of the rover in the map frame over a window of time

        R2d rover_velocity_sum = R2d::Zero();
        double rover_heading_change = 0.0;
        std::size_t readings = 0;

        rclcpp::Time end = get_clock()->now();
        rclcpp::Time start = end - WINDOW;

        for (rclcpp::Time t = start; t < end; t += STEP) {

            try {
                auto rover_in_map_old = SE3Conversions::fromTfTree(tf_buffer, rover_frame, world_frame, t - STEP);
                auto rover_in_map_new = SE3Conversions::fromTfTree(tf_buffer, rover_frame, world_frame, t);
                R3d rover_velocity_in_map = (rover_in_map_new.translation() - rover_in_map_old.translation()) / STEP.seconds();
                R3d rover_angular_velocity_in_map = (rover_in_map_new.asSO3() - rover_in_map_old.asSO3()).coeffs();
                rover_velocity_sum += rover_velocity_in_map.head<2>();
                rover_heading_change += std::fabs(rover_angular_velocity_in_map.z());
                ++readings;
            } catch (tf2::ConnectivityException const& e) {
                RCLCPP_WARN_STREAM(get_logger(), e.what());
                return;
            } catch (tf2::LookupException const& e) {
                RCLCPP_WARN_STREAM(get_logger(), e.what());
                return;
            } catch (tf2::ExtrapolationException const&) { }
        }

        // 2. Ensure the rover has actually moved to avoid correcting while standing still

        if (R2d mean_velocity_in_map = rover_velocity_sum / static_cast<double>(readings); mean_velocity_in_map.norm() < MIN_LINEAR_SPEED) {
            RCLCPP_INFO_STREAM(get_logger(), std::format("Rover is not moving fast enough: speed = {} m/s", mean_velocity_in_map.norm()));
            return;
        }

        if (rover_heading_change > MAX_ANGULAR_CHANGE) {
            RCLCPP_INFO_STREAM(get_logger(), std::format("Rover is not moving straight enough: heading change = {} rad", rover_heading_change));
            return;
        }

        double corrected_heading_in_map = std::atan2(rover_velocity_sum.y(), rover_velocity_sum.x());

        SO3d uncorrected_orientation = ros_quat_to_eigen_quat(current_imu_uncalib->orientation);
        R2d uncorrected_forward = uncorrected_orientation.rotation().col(0).head<2>();
        double estimated_heading_in_map = std::atan2(uncorrected_forward.y(), uncorrected_forward.x());

        double heading_correction_delta = corrected_heading_in_map - estimated_heading_in_map;

        correction_rotation = Eigen::AngleAxisd{heading_correction_delta, R3d::UnitZ()};

        RCLCPP_INFO_STREAM(get_logger(), std::format("Correcting heading by: {}", correction_rotation->z()));
    }
} // namespace mrover


int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::PoseFilter>());
    rclcpp::shutdown();
    return 0;
}


