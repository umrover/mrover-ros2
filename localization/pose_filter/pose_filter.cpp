#include "pose_filter.hpp"

namespace mrover {
    

    auto PoseFilter::ros_quat_to_eigen_quat(geometry_msgs::msg::Quaternion const& q) -> Eigen::Quaterniond {
        return {q.w, q.x, q.y, q.z};
    }

    PoseFilter::PoseFilter() : Node("pose_filter") {

        linearized_position_sub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("/linearized_position", 1, [&](const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &position) {
            last_position = *position;
        });
        imu_sub_.subscribe(this, "/zed_imu/data_raw");
        mag_heading_sub_.subscribe(this, "/zed_imu/mag_heading");
        rtk_heading_sub_.subscribe(this, "/heading/fix");
        rtk_heading_fix_status_sub_.subscribe(this, "/heading_fix_status");

        odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 1);
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, [&](geometry_msgs::msg::Twist::ConstSharedPtr const& twist) {
            twists.push_back(*twist);
        });

        imu_watchdog_timeout = this->create_wall_timer(IMU_WATCHDOG_TIMEOUT.to_chrono<std::chrono::milliseconds>(), [&]() {
            RCLCPP_WARN(get_logger(), "ZED IMU data timed out!");
            last_imu.reset();
        })

        rtk_sync_ = std::make_shared<message_filters::Synchronizer<RTKSyncPolicy>>(
            RTKSyncPolicy(10), rtk_heading_sub_, rtk_fix_status_sub_);
        rtk_sync_->setAgePenalty(0.5); 
        rtk_sync_->registerCallback(&PoseFilter::rtk_heading_callback, this);

        imu_and_mag_sync_ = std::make_shared<message_filters::Synchronizer<ImuMagSyncPolicy>>(
            ImuMagSyncPolicy(10), imu_sub_, mag_heading_sub_);
        imu_and_mag_sync_->setAgePenalty(0.5);  // Match original configuration
        imu_and_mag_sync_->registerCallback(&PoseFilter::imu_and_mag_callback, this);
    
        correction_timer = this->create_wall_timer(WINDOW.to_chrono<std::chrono::milliseconds>(), [this]() -> void {
            correction_timer_callback();
        });

        declare_parameter("rover_frame", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("world_frame", rclcpp::ParameterType::PARAMETER_STRING);

        rover_frame = get_parameter("rover_frame").as_string();
        world_frame = get_parameter("world_frame").as_string();

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

    void PoseFilter::rtk_heading_callback(const mrover::msg::Heading::ConstSharedPtr const& rtk_heading_msg, const mrover::msg::FixStatus::ConstSharedPtr const& rtk_fix_status_msg) {
        last_rtk_heading = *rtk_heading_msg;
        last_rtk_fix_status = *rtk_fix_status_msg;
    }

    void PoseFilter::imu_and_mag_callback(const mrover::msg::Imu::ConstSharedPtr const& imu_msg, const mrover::msg::Heading::ConstSharedPtr &mag_heading) {
        imu_watchdog_timeout->reset();
        last_imu = *imu_msg;
        last_mag_heading = *mag_heading;
    }

    void PoseFilter::pose_sub_callback(geometry_msgs::msg::Vector3Stamped::ConstSharedPtr const& linearized_pos_msg) {
       
        if ((!last_position.has_value()) || (!last_imu.has_value()) || (!last_rtk_heading.has_value()) || (!last_rtk_fix_status.has_value())) {
            RCLCPP_WARN(get_logger(), "Missing Last position, IMU, rtk heading or rtk heading fix status data. Ignoring pose correction via RTK heading.");
            return;
        }
 
        if ((last_rtk_fix_status->fix_type.fix != mrover::msg::FixType::FIXED)) { 
            RCLCPP_WARN(get_logger(), "Invalid heading fix status. Ignoring pose correction via DA-RTK Heading");
            return;
         }
 
        Eigen::Vector3d position_in_map(last_position->vector.x, last_position->vector.y, last_position->vector.z);
        Eigen::Quaterniond uncorrected_quaternion = ros_quat_to_eigen_quat(last_imu->orientation);
        R2d uncorrected_forward = uncorrected_quaternion.toRotationMatrix().col(0).head<2>();
        double uncorrected_heading = std::atan2(uncorrected_forward.y(), uncorrected_forward.x());

        if (last_rtk_fix_status->fix_type.fix == mrover::msg::FixType::FIXED) {
            double measured_rtk_heading_deg = fmod(last_rtk_heading->heading + 90.0, 360.0);
            if (measured_rtk_heading_deg > 270) {
                measured_rtk_heading_deg -= 360;
            }
            double measured_rtk_heading = (90.0 - measured_rtk_heading_deg) * (M_PI / 180.0);
            double rtk_heading_correction_delta = measured_rtk_heading - uncorrected_heading; //heading difference: rtk heading + IMU
            curr_heading_correction = Eigen::AngleAxisd(rtk_heading_correction_delta, Eigen::Vector3d::UnitZ());
        } else if (last_mag_heading && curr_heading_correction) {
            double measured_mag_heading_deg = last_mag_heading->heading;
            measured_mag_heading_deg = 90.0 - measured_mag_heading_deg;
            if (measured_mag_heading_deg < -180.0) {
                measured_mag_heading_deg = 360 + measured_mag_heading_deg;
            }
            double measured_mag_heading = measured_mag_heading_deg * (M_PI / 180);

            double curr_mag_correction_delta = measured_mag_heading - uncorrected_heading; //heading difference: magnetometer + IMU
            double prev_rtk_heading_correction_delta = curr_heading_correction->angle(); 

            if (std::abs(prev_rtk_heading_correction_delta - curr_mag_correction_delta) < HEADING_THRESHOLD) {
                curr_heading_correction = Eigen::AngleAxisd(curr_mag_correction_delta, Eigen::Vector3d::UnitZ());
            }
        }

        pose_callback_heading = curr_heading_correction.value();
        R2d pose_heading_correction_forward = curr_heading_correction.toRotationMatrix().col(0).head(2);
        pose_heading_correction_delta = std::atan2(pose_heading_correction_forward.y(). pose_heading_correction_forward.x());

        compare_and_select_heading();

        double final_heading_value = averaged_pose_and_correction_heading_.value_or(curr_heading_correction_.value());

        R3d position_in_map(last_position->vector.x, last_position->vector.y, last_position->vector.z);
        SE3d pose_in_map(position_in_map, SO3d::Identity());

        Eigen::Quaterniond uncorrected_orientation = ros_quat_to_eigen_quat(last_imu->orientation);
        SO3d uncorrected_orientation_rotm = uncorrected_orientation;
        SO3d corrected_orientation = Eigen::AngleAxisd(final_heading_value, Eigen::Vector3d::UnitZ()) * uncorrected_orientation_rotm;
        pose_in_map.so3() = corrected_orientation;
        SE3Conversion::pushToTfTree(tf_broadcaster, rover_frame, world_frame, pose_in_map, getClock()->now());
 
        RCLCPP_INFO(get_logger(), "Published to TF Tree: Position (%f, %f, %f), Orientation (%f, %f, %f, %f)",
             position_in_map.x(), position_in_map.y(), position_in_map.z(),
             corrected_orientation.x(), corrected_orientation.y(), corrected_orientation.z(), corrected_orientation.w()
         );
     }

    void PoseFilter::compare_and_select_heading() {

        if ((!correcton_timer_heading.has_value()) || (!pose_callback_heading.has_value()) || (!pose_heading_correction_delta.has_value()) || (!correction_timer_heading_delta.has_value())) {
            RCLCPP_WARN(get_logger(), "Missing heading values");
            return;
        }
        
        if ((std::abs(pose_heading_correction_delta.value() - correction_timer_heading_delta.value())) < HEADING_THRESHOLD) {
            RCLCPP_DEBUG(get_logger(), "Heading deltas are within acceptable range (%.3f < %.3f). No averaging needed.", pose_heading_correction_delta.value() - correction_timer_heading_delta.value(), HEADING_THRESHOLD)
            return;
        } else {
            RCLCPP_WARN(get_logger(), "Heading delta's exceeds threshold. Now calculating the averaged heading.");
            averaged_pose_and_correction_heading = ((pose_callback_heading.value()) + (corection_timer_heading.value())) / 2.0;
            return;
        }
        }
    }

        SE3d::Tangent twist; 
        if (last_pose_in_map && last_pose_time) {
            twist = (pose_in_map - last_pose_in_map.value()) / ((linearized_pos_msg->header.stamp.sec + linearized_pos_msg->header.stamp.nanosec * 10e-9) - (last_pose_time->sec + last_pose_time->nanosec * 10e-9));
        }

        nav_msgs::msg::Odometry odometry;
        odometry.header = linearized_pos_msg->header;
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
        last_pose_time = linearized_pos_msg->header.stamp;


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

        correction_timer_heading = corrected_heading_in_map;

        SO3d uncorrected_orientation = ros_quat_to_eigen_quat(current_imu->orientation);
        R2d uncorrected_forward = uncorrected_orientation.rotation().col(0).head<2>();
        double estimated_heading_in_map = std::atan2(uncorrected_forward.y(), uncorrected_forward.x());

        double heading_correction_delta = corrected_heading_in_map - estimated_heading_in_map; 

        correction_timer_heading_delta = heading_correction_delta;

        compare_and_select_heading();

        correction_rotation = Eigen::AngleAxisd{heading_correction_delta, R3d::UnitZ()};

        RCLCPP_INFO_STREAM(get_logger(), std::format("Correcting heading by: {}", correction_rotation->z()));
    }
  // namespace mrover


int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::PoseFilter>());
    rclcpp::shutdown();
    return 0;
}

//make angular velocity like a ros parameter (have a threshold for if the rover is driving straight enough)
//adjust parameters in localisation.yaml.