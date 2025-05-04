#pragma once

#include "gst_utils.hpp"
#include "pch.hpp"

// Uses gstreamer to encode
// The input can be a ROS BGRA image topic or a USB device
// Hardware accelerated is used when possible (with the Jetson or NVIDIA GPUs)
// Run "export GST_DEBUG=2" to debug gstreamer issues

namespace mrover {

    class GstV4L2Encoder final : public rclcpp::Node {

        rclcpp::Service<srv::MediaControl>::SharedPtr mMediaControlServer;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mImageCaptureServer;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mImagePublisher;

        // For example, /dev/video0
        // These device paths are not garunteed to stay the same between reboots
        // Prefer sys path for non-debugging purposes
        std::string mDeviceNode;

        bool mDisableAutoWhiteBalance{}; // Useful for science, the UV LEDs can mess with the white balance

        bool mCropEnabled{};
        int mCropLeft{}, mCropRight{}, mCropTop{}, mCropBottom{};

        gst::video::v4l2::CaptureFormat mStreamCaptureFormat;
        gst::video::Codec mCodec;
        std::uint64_t mBitrate;
        std::string mAddress;
        std::uint16_t mPort;
        gst::PipelineWrapper mStreamPipelineWrapper;


        bool mImageCaptureEnabled;
        gst::video::v4l2::CaptureFormat mImageCaptureFormat;
        std::string mImageCapturePipelineLaunch;

        GMainLoop* mMainLoop{};
        std::thread mMainLoopThread;

        auto createStreamPipeline() -> void;
        auto createImageCapturePipeline() -> void;

        auto initStreamPipeline() -> void;

        auto mediaControlServerCallback(srv::MediaControl::Request::SharedPtr req, srv::MediaControl::Response::SharedPtr res) -> void;
        auto imageCaptureServerCallback(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res) -> void;

    public:
        // __attribute__ ((visibility("default")))
        explicit GstV4L2Encoder(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        ~GstV4L2Encoder() override;
    };

} // namespace mrover
