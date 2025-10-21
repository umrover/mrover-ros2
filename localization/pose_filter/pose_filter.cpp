#include "pose_filter.hpp"

namespace mrover {

    PoseFilter::PoseFilter() : Node("pose_filter") {

        declare_parameter("world_frame", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("gps_frame", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("rover_heading_change_threshold", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("minimum_linear_speed", rclcpp::ParameterType::PARAMETER_DOUBLE);

        world_frame = get_parameter("world_frame").as_string();
        gps_frame = get_parameter("gps_frame").as_string();
        rover_heading_change_threshold = get_parameter("rover_heading_change_threshold").as_double();
        minimum_linear_speed = get_parameter("minimum_linear_speed").as_double();

        // subscribers
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/zed_imu/data_raw", 1, [&](const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg) {
            imu_callback(*imu_msg);
        });

        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, [&](const geometry_msgs::msg::Twist::ConstSharedPtr& twist_msg) {
            twists.push_back(*twist_msg);
        });

        pos_sub.subscribe(this, "/linearized_position");
        pos_status_sub.subscribe(this, "/gps_fix_status");
        
        // synchronise measurements 
        pos_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::Vector3Stamped, mrover::msg::FixStatus>>>(
            message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::Vector3Stamped, mrover::msg::FixStatus>(10),
            pos_sub,
            pos_status_sub
        );

        pos_sync->setAgePenalty(0.5);
        pos_sync->registerCallback(&PoseFilter::pos_callback, this);

        imu_watchdog = this->create_wall_timer(IMU_WATCHDOG_TIMEOUT.to_chrono<std::chrono::milliseconds>(), [&]() {
            RCLCPP_WARN(get_logger(), "ZED IMU timed out!");
            last_imu.reset();
        });

        correction_timer = this->create_wall_timer(WINDOW.to_chrono<std::chrono::milliseconds>(), [this]() -> void {
            drive_forward_callback();
        });
    }

    void PoseFilter::pos_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &pos_msg, const mrover::msg::FixStatus::ConstSharedPtr& pos_status_msg) {
        if (pos_status_msg->fix_type.fix == mrover::msg::FixType::NO_SOL) {
            return;
        }

        last_pos = pos_msg->vector;
    }

    void PoseFilter::imu_callback(const sensor_msgs::msg::Imu& imu_msg) {
        imu_watchdog.reset();

        last_imu = imu_msg;

        if (!last_pos) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No position data!");
            return;
        }

        R3d position_in_map(last_pos.value().x, last_pos.value().y, last_pos.value().z);
        SE3d pose_in_map(position_in_map, SO3d::Identity());

        Eigen::Quaterniond uncorrected_orientation(imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z);
        SO3d uncorrected_orientation_rotm = uncorrected_orientation;

        if (!curr_heading_correction) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No heading correction, publishing raw IMU");
            pose_in_map.asSO3() = uncorrected_orientation_rotm;
        }
        else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Heading corrected by: {%f}", curr_heading_correction->z());
            SO3d corrected_orientation = curr_heading_correction.value() * uncorrected_orientation_rotm;
            pose_in_map.asSO3() = corrected_orientation;
        }

        SE3Conversions::pushToTfTree(tf_broadcaster, gps_frame, world_frame, pose_in_map, get_clock()->now());
    }

    void PoseFilter::drive_forward_callback() {

        if (!last_imu) {
            RCLCPP_WARN(get_logger(), "No IMU!");
            return;
        }

        double mean_cmd_vel = 0;
        for (auto & twist : twists) {
            mean_cmd_vel += twist.linear.x / static_cast<double>(twists.size());
        }
        twists.clear();
        if (mean_cmd_vel < minimum_linear_speed) {
            RCLCPP_WARN(get_logger(), "Rover is not being commanded forward!");
            return;
        }
        
        R2d rover_velocity_sum = R2d::Zero();
        double rover_heading_change = 0.0;
        std::size_t readings = 0;

        rclcpp::Time end = get_clock()->now();
        rclcpp::Time start = end - WINDOW;

        for (rclcpp::Time t = start; t < end; t += STEP) {

            try {
                auto rover_in_map_old = SE3Conversions::fromTfTree(tf_buffer, gps_frame, world_frame, t - STEP);
                auto rover_in_map_new = SE3Conversions::fromTfTree(tf_buffer, gps_frame, world_frame, t);
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
            } catch (tf2::ExtrapolationException const& e) {
                RCLCPP_WARN_STREAM(get_logger(), e.what());
                return;
            }
        }

        if (rover_heading_change > rover_heading_change_threshold) {
            RCLCPP_WARN(get_logger(), "Rover is not moving straight enough: Heading change = {%f} deg", rover_heading_change * (180 / M_PI));
            return;
        }

        R2d mean_velocity = rover_velocity_sum / static_cast<double>(readings);
        if (mean_velocity.norm() < minimum_linear_speed) {
            RCLCPP_WARN(get_logger(), "Rover stationary - insufficient speed: %.3f m/s", mean_velocity.norm());
            return;
        }
        
        double drive_forward_heading = atan2(rover_velocity_sum.y(), rover_velocity_sum.x());

        RCLCPP_INFO(get_logger(), "Drive forward heading: %f deg", drive_forward_heading * (180 / M_PI));

        Eigen::Quaterniond uncorrected_orientation(last_imu.value().orientation.w, last_imu.value().orientation.x, last_imu.value().orientation.y, last_imu.value().orientation.z);
        R2d uncorrected_forward = uncorrected_orientation.toRotationMatrix().col(0).head(2);
        double uncorrected_heading = std::atan2(uncorrected_forward.y(), uncorrected_forward.x());

        double heading_correction_delta = drive_forward_heading - uncorrected_heading;
        curr_heading_correction = Eigen::AngleAxisd(heading_correction_delta, R3d::UnitZ());


    }

    
}

int main(int argc, char**argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::PoseFilter>());
    rclcpp::shutdown();
    return 0;

}
