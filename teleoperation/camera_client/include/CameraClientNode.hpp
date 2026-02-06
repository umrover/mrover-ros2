#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <QImage>
#include <QObject>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mrover/srv/media_control.hpp>

namespace mrover {

    struct CameraInfo {
        std::string name;
        std::string pipeline;
    };

    // inherits from both QObject (for signals) and rclcpp::Node (for ROS)
    class CameraClientNode : public QObject, public rclcpp::Node {
        Q_OBJECT

        std::unordered_map<std::string, rclcpp::Client<srv::MediaControl>::SharedPtr> mMediaControlClients;
        std::unordered_map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> mImageCaptureClients;
        std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> mImageCaptureSubscribers;

        auto imageCaptureCallback(std::string const& cameraName, sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;
        auto sendMediaControlRequest(std::string const& cameraName, std::uint8_t command) -> bool;
        auto sendScreenshotRequest(std::string const& cameraName) -> bool;

    public:
        explicit CameraClientNode();

        // call after connecting signals to discover cameras from parameters
        auto discoverCameras() -> void;

    signals:
        void cameraDiscovered(CameraInfo info);
        void imageCaptured(QString cameraName, QImage image);

    public slots:
        bool requestPause(std::string const& cameraName);
        bool requestPlay(std::string const& cameraName);
        bool requestStop(std::string const& cameraName);
        bool requestScreenshot(std::string const& cameraName);
    };

} // namespace mrover
