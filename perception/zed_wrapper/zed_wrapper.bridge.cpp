#include "zed_wrapper.hpp"
#include "mrover/msg/detail/fix_status__struct.hpp"
#include "mrover/msg/detail/fix_type__struct.hpp"

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

    auto fillMagMessage(sl::SensorsData::MagnetometerData const& magData, sensor_msgs::msg::MagneticField& magMsg, mrover::msg::Heading &headingMsg, mrover::msg::FixStatus &headingStatusMsg) -> void {
        magMsg.magnetic_field.x = magData.magnetic_field_calibrated.x;
        magMsg.magnetic_field.y = magData.magnetic_field_calibrated.y;
        magMsg.magnetic_field.z = magData.magnetic_field_calibrated.z;

        magMsg.magnetic_field_covariance.fill(0.0f);
        magMsg.magnetic_field_covariance[0] = 0.039e-6;
        magMsg.magnetic_field_covariance[4] = 0.037e-6;
        magMsg.magnetic_field_covariance[8] = 0.047e-6;

        headingMsg.heading = magData.magnetic_heading;
        if (magData.magnetic_heading_state == sl::SensorsData::MagnetometerData::HEADING_STATE::GOOD) {
            headingStatusMsg.fix_type.fix = mrover::msg::FixType::FIXED;
        }
        else if (magData.magnetic_heading_state == sl::SensorsData::MagnetometerData::HEADING_STATE::OK) {
            headingStatusMsg.fix_type.fix = mrover::msg::FixType::FLOAT;
        }
        else {
            headingStatusMsg.fix_type.fix = mrover::msg::FixType::NONE;
        }
    }

    auto fillImageMessage(sl::Mat const& bgra, sensor_msgs::msg::Image::UniquePtr const& msg) -> void {
        assert(bgra.getChannels() == 4);
        assert(msg);

        msg->height = bgra.getHeight();
        msg->width = bgra.getWidth();
        msg->encoding = sensor_msgs::image_encodings::BGRA8;
        msg->step = bgra.getStepBytes();
        msg->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        auto* bgrGpuPtr = bgra.getPtr<sl::uchar1>(sl::MEM::GPU);
        size_t size = msg->step * msg->height;
        msg->data.resize(size);
        checkCudaError(cudaMemcpy(msg->data.data(), bgrGpuPtr, size, cudaMemcpyDeviceToHost));
    }

    auto fillCameraInfoMessages(sl::CalibrationParameters& calibration, sl::Resolution const& resolution,
                                sensor_msgs::msg::CameraInfo& leftInfoMsg, sensor_msgs::msg::CameraInfo& rightInfoMsg) -> void {


        leftInfoMsg.width = resolution.width;
        leftInfoMsg.height = resolution.height;
        rightInfoMsg.width = resolution.width;
        rightInfoMsg.height = resolution.height;
        leftInfoMsg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        rightInfoMsg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        leftInfoMsg.d.resize(5);
        rightInfoMsg.d.resize(5);
        leftInfoMsg.d[0] = calibration.left_cam.disto[0];
        leftInfoMsg.d[1] = calibration.left_cam.disto[1];
        leftInfoMsg.d[2] = calibration.left_cam.disto[4];
        leftInfoMsg.d[3] = calibration.left_cam.disto[2];
        leftInfoMsg.d[4] = calibration.left_cam.disto[3];
        rightInfoMsg.d[0] = calibration.right_cam.disto[0];
        rightInfoMsg.d[1] = calibration.right_cam.disto[1];
        rightInfoMsg.d[2] = calibration.right_cam.disto[4];
        rightInfoMsg.d[3] = calibration.right_cam.disto[2];
        rightInfoMsg.d[4] = calibration.right_cam.disto[3];
        leftInfoMsg.k.fill(0.0);
        rightInfoMsg.k.fill(0.0);
        leftInfoMsg.k[0] = calibration.left_cam.fx;
        leftInfoMsg.k[2] = calibration.left_cam.cx;
        leftInfoMsg.k[4] = calibration.left_cam.fy;
        leftInfoMsg.k[5] = calibration.left_cam.cy;
        leftInfoMsg.k[8] = 1.0;
        rightInfoMsg.k[0] = calibration.right_cam.fx;
        rightInfoMsg.k[2] = calibration.right_cam.cx;
        rightInfoMsg.k[4] = calibration.right_cam.fy;
        rightInfoMsg.k[5] = calibration.right_cam.cy;
        rightInfoMsg.k[8] = 1;
        leftInfoMsg.r.fill(0.0);
        rightInfoMsg.r.fill(0.0);
        for (size_t i = 0; i < 3; ++i) {
            leftInfoMsg.r[i * 3 + i] = 1.0;
            rightInfoMsg.r[i * 3 + i] = 1.0;
        }
        leftInfoMsg.p.fill(0.0);
        rightInfoMsg.p.fill(0.0);
        leftInfoMsg.p[0] = calibration.left_cam.fx;
        leftInfoMsg.p[2] = calibration.left_cam.cx;
        leftInfoMsg.p[5] = calibration.left_cam.fy;
        leftInfoMsg.p[6] = calibration.left_cam.cy;
        leftInfoMsg.p[10] = 1.0;
        rightInfoMsg.p[3] = -1.0 * calibration.left_cam.fx * calibration.getCameraBaseline();
        rightInfoMsg.p[0] = calibration.right_cam.fx;
        rightInfoMsg.p[2] = calibration.right_cam.cx;
        rightInfoMsg.p[5] = calibration.right_cam.fy;
        rightInfoMsg.p[6] = calibration.right_cam.cy;
        rightInfoMsg.p[10] = 1.0;
    }
}; // namespace mrover
