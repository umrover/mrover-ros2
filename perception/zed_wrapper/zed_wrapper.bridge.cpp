#include "zed_wrapper.hpp"

namespace mrover {

	constexpr static float DEG_TO_RAD = std::numbers::pi_v<float> / 180.0f;
    constexpr static std::uint64_t NS_PER_S = 1000000000;

	auto slTime2Ros(sl::Timestamp t) -> rclcpp::Time {
        return {static_cast<int32_t>(t.getNanoseconds() / NS_PER_S),
                static_cast<uint32_t>(t.getNanoseconds() % NS_PER_S)};
    }

	auto fillImuMessage(rclcpp::Node* node, sl::SensorsData::IMUData& imuData, sensor_msgs::msg::Imu& msg) -> void {
        msg.header.stamp = node->now();
        msg.orientation.x = imuData.pose.getOrientation().x;
        msg.orientation.y = imuData.pose.getOrientation().y;
        msg.orientation.z = imuData.pose.getOrientation().z;
        msg.orientation.w = imuData.pose.getOrientation().w;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                msg.orientation_covariance[i * 3 + j] = imuData.pose_covariance(i, j) * DEG_TO_RAD * DEG_TO_RAD;
        msg.angular_velocity.x = imuData.angular_velocity.x * DEG_TO_RAD;
        msg.angular_velocity.y = imuData.angular_velocity.y * DEG_TO_RAD;
        msg.angular_velocity.z = imuData.angular_velocity.z * DEG_TO_RAD;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                msg.angular_velocity_covariance[i * 3 + j] = imuData.angular_velocity_covariance(i, j) * DEG_TO_RAD * DEG_TO_RAD;
        msg.linear_acceleration.x = imuData.linear_acceleration.x;
        msg.linear_acceleration.y = imuData.linear_acceleration.y;
        msg.linear_acceleration.z = imuData.linear_acceleration.z;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                msg.linear_acceleration_covariance[i * 3 + j] = imuData.linear_acceleration_covariance(i, j);
    }

    auto fillMagMessage(sl::SensorsData::MagnetometerData const& magData, sensor_msgs::msg::MagneticField& msg) -> void {
        msg.magnetic_field.x = magData.magnetic_field_calibrated.x;
        msg.magnetic_field.y = magData.magnetic_field_calibrated.y;
        msg.magnetic_field.z = magData.magnetic_field_calibrated.z;

        msg.magnetic_field_covariance.fill(0.0f);
        msg.magnetic_field_covariance[0] = 0.039e-6;
        msg.magnetic_field_covariance[4] = 0.037e-6;
        msg.magnetic_field_covariance[8] = 0.047e-6;
    }
};
