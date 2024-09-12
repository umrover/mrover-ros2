#pragma once

#include "pch.hpp"

// Uses gstreamer to encode and stream video over a websocket
// The input can be a ROS BGRA image topic or a USB device
// Hardware accelerated is used when possible (with the Jetson or NVIDIA GPUs)
// Run "export GST_DEBUG=2" to debug gstreamer issues

namespace mrover {

    struct ChunkHeader {
        enum struct Resolution : std::uint8_t {
            VGA, // 640x480
            HD,  // 1280x720
            FHD, // 1920x1080
        } resolution;
        enum struct Codec : std::uint8_t {
            H265,
            H264,
        } codec;
    };

    class GstWebsocketStreamer final : public rclcpp::Node {

        bool mDecodeJpegFromDevice{};    // Uses less USB hub bandwidth, which is limited since we are using 2.0
        bool mDisableAutoWhiteBalance{}; // Useful for science, the UV LEDs can mess with the white balance
        std::string mImageTopic;
        std::string mDeviceDescriptor;
        std::uint64_t mBitrate{};
        std::uint16_t mImageWidth{}, mImageHeight{}, mImageFramerate{};

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSubscription;

        std::optional<WebsocketServer> mStreamServer;

        GstElement *mImageSource{}, *mStreamSink{}, *mPipeline{};
        GMainLoop* mMainLoop{};
        std::thread mMainLoopThread;
        std::thread mStreamSinkThread;

        ChunkHeader mChunkHeader{};

        auto pullStreamSamplesLoop() -> void;

        auto initPipeline(std::string_view deviceNode) -> void;

        auto imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;

    public:
        // __attribute__ ((visibility("default")))
        explicit GstWebsocketStreamer(const rclcpp::NodeOptions &options);

        ~GstWebsocketStreamer() override;
    };

} // namespace mrover
