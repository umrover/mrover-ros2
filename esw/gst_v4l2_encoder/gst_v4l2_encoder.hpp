#pragma once

#include "pch.hpp"

// Uses gstreamer to encode
// The input can be a ROS BGRA image topic or a USB device
// Hardware accelerated is used when possible (with the Jetson or NVIDIA GPUs)
// Run "export GST_DEBUG=2" to debug gstreamer issues

namespace mrover {

    class GstV4L2Encoder final : public rclcpp::Node {

        bool mDecodeJpegFromDevice{};    // Uses less USB hub bandwidth, which is limited since we are using 2.0
        bool mDisableAutoWhiteBalance{}; // Useful for science, the UV LEDs can mess with the white balance
        std::string mDeviceDescriptor;
        std::uint64_t mBitrate{};
        std::uint16_t mImageWidth{}, mImageHeight{}, mImageFramerate{};
        std::string mAddress;
        std::uint16_t mPort;

        bool mCropEnabled{};
        int mCropLeft{}, mCropRight{}, mCropTop{}, mCropBottom{};

        GstElement* mPipeline{};
        GMainLoop* mMainLoop{};
        std::thread mMainLoopThread;

        auto initPipeline(std::string_view deviceNode) -> void;

    public:
        // __attribute__ ((visibility("default")))
        explicit GstV4L2Encoder(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        ~GstV4L2Encoder() override;
    };

} // namespace mrover
