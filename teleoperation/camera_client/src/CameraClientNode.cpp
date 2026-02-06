#include "CameraClientNode.hpp"

#include <chrono>
#include <format>

#include <QDebug>

#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <gst_utils.hpp>

#include "GstRtpVideoCreatorWidget.hpp"

namespace mrover {

    CameraClientNode::CameraClientNode()
        : QObject(nullptr),
          Node("camera_client") {

        RCLCPP_INFO(get_logger(), "Camera client initialized");
    }

    auto CameraClientNode::discoverCameras() -> void {
        declare_parameter("cameras", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
        auto cameraNames = get_parameter("cameras").as_string_array();

        declare_parameter("rtp_jitter_ms", 100);
        auto rtpJitterMs = std::chrono::milliseconds(get_parameter("rtp_jitter_ms").as_int());

        for (auto const& cameraName: cameraNames) {
            RCLCPP_INFO(get_logger(), "cameraName: %s", cameraName.c_str());

            if (mMediaControlClients.contains(cameraName)) {
                RCLCPP_WARN(get_logger(), "Camera %s already exists, skipping", cameraName.c_str());
                continue;
            }

            declare_parameter(std::format("{}.port", cameraName), rclcpp::ParameterType::PARAMETER_INTEGER);
            auto const port = static_cast<std::uint16_t>(get_parameter(std::format("{}.port", cameraName)).as_int());

            declare_parameter(std::format("{}.stream.codec", cameraName), rclcpp::ParameterType::PARAMETER_STRING);
            std::string const codec = get_parameter(std::format("{}.stream.codec", cameraName)).as_string();

            std::string const pipeline = createRtpToRawSrc(port, gst::video::getCodecFromStringView(codec), rtpJitterMs);

            mMediaControlClients.emplace(cameraName, create_client<srv::MediaControl>(std::format("{}_media_control", cameraName)));
            mImageCaptureClients.emplace(cameraName, create_client<std_srvs::srv::Trigger>(std::format("{}_image_capture", cameraName)));
            mImageCaptureSubscribers.emplace(cameraName, create_subscription<sensor_msgs::msg::Image>(
                                                                 std::format("{}_image", cameraName), 1,
                                                                 [this, cameraName](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
                                                                     imageCaptureCallback(cameraName, msg);
                                                                 }));

            // emit signal for GUI to handle camera setup
            emit cameraDiscovered(CameraInfo{.name = cameraName, .pipeline = pipeline});
        }
    }

    auto CameraClientNode::sendMediaControlRequest(std::string const& cameraName, std::uint8_t command) -> bool {
        auto it = mMediaControlClients.find(cameraName);
        if (it == mMediaControlClients.end()) {
            RCLCPP_ERROR(get_logger(), "Camera %s not found", cameraName.c_str());
            return false;
        }
        auto request = std::make_shared<srv::MediaControl::Request>();
        request->command = command;
        it->second->async_send_request(request);
        return true;
    }

    auto CameraClientNode::sendScreenshotRequest(std::string const& cameraName) -> bool {
        auto it = mImageCaptureClients.find(cameraName);
        if (it == mImageCaptureClients.end()) {
            RCLCPP_ERROR(get_logger(), "Camera %s not found", cameraName.c_str());
            return false;
        }
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        it->second->async_send_request(request);
        return true;
    }

    bool CameraClientNode::requestPause(std::string const& cameraName) {
        qDebug() << "Pause request for camera" << cameraName.c_str();
        return sendMediaControlRequest(cameraName, srv::MediaControl::Request::PAUSE);
    }

    bool CameraClientNode::requestPlay(std::string const& cameraName) {
        qDebug() << "Play request for camera" << cameraName.c_str();
        return sendMediaControlRequest(cameraName, srv::MediaControl::Request::PLAY);
    }

    bool CameraClientNode::requestStop(std::string const& cameraName) {
        qDebug() << "Stop request for camera" << cameraName.c_str();
        return sendMediaControlRequest(cameraName, srv::MediaControl::Request::STOP);
    }

    bool CameraClientNode::requestScreenshot(std::string const& cameraName) {
        qDebug() << "Screenshot request for camera" << cameraName.c_str();
        return sendScreenshotRequest(cameraName);
    }

    auto CameraClientNode::imageCaptureCallback(std::string const& cameraName, sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void {
        RCLCPP_INFO(get_logger(), "Received image from camera");
        if (msg->encoding != sensor_msgs::image_encodings::BGR8) {
            RCLCPP_ERROR(get_logger(), "Unsupported encoding - image capture must be BGR8");
            return;
        }

        cv::Size receivedSize{static_cast<int>(msg->width), static_cast<int>(msg->height)};
        cv::Mat bgrFrame{receivedSize, CV_8UC3, const_cast<std::uint8_t*>(msg->data.data()), msg->step};

        // create a deep copy of the image data for the signal
        QImage qImg(bgrFrame.data, bgrFrame.cols, bgrFrame.rows, static_cast<int>(bgrFrame.step), QImage::Format_BGR888);

        // emit signal with a copy (since the original data will go out of scope)
        emit imageCaptured(QString::fromStdString(cameraName), qImg.copy());
    }

} // namespace mrover
