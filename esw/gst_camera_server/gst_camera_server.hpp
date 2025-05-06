#pragma once

#include "pch.hpp"

// Uses gstreamer to encode
// The input can be a ROS BGRA image topic or a USB device
// Hardware accelerated is used when possible (with the Jetson or NVIDIA GPUs)
// Run "export GST_DEBUG=2" to debug gstreamer issues

namespace mrover {

    class GstCameraServer final : public rclcpp::Node {

        rclcpp::Service<srv::MediaControl>::SharedPtr mMediaControlServer;

        // For example, /dev/video0
        // These device paths are not garunteed to stay the same between reboots
        // Prefer sys path for non-debugging purposes
        std::string mDeviceNode;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mDeviceImageSubscriber;

        bool mDisableAutoWhiteBalance{}; // Useful for science, the UV LEDs can mess with the white balance

        bool mCropEnabled{};
        int mCropLeft{}, mCropRight{}, mCropTop{}, mCropBottom{};

        gst::video::v4l2::CaptureFormat mStreamCaptureFormat;
        gst::video::Codec mCodec;
        std::uint64_t mBitrate;
        std::string mAddress;
        std::uint16_t mPort;
        gst::PipelineWrapper mStreamPipelineWrapper;
        GstElement* mStreamDeviceImageSource;

        bool mImageCaptureEnabled;
        gst::video::v4l2::CaptureFormat mImageCaptureFormat;
        std::string mImageCapturePipelineLaunch;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mImageCaptureServer;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mImageCapturePublisher;
        GstElement* mImageCaptureDeviceImageSource;

        GMainLoop* mMainLoop{};
        std::thread mMainLoopThread;

        auto deviceImageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;
        auto createStreamPipeline() -> void;
        auto createImageCapturePipeline() -> void;

        auto initStreamPipelines() -> void;

        auto mediaControlServerCallback(srv::MediaControl::Request::ConstSharedPtr const& req, srv::MediaControl::Response::SharedPtr const& res) -> void;
        auto imageCaptureServerCallback(std_srvs::srv::Trigger::Request::ConstSharedPtr const&, std_srvs::srv::Trigger::Response::SharedPtr const& res) -> void;

    public:
        // __attribute__ ((visibility("default")))
        explicit GstCameraServer(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        ~GstCameraServer() override;
    };

} // namespace mrover
