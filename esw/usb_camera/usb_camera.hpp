#pragma once

#include "pch.hpp"

namespace mrover {

    class UsbCamera final : public rclcpp::Node {

        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mCamInfoPub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mImgPub;

        std::uint16_t mWidth{}, mHeight{};

        GstElement *mStreamSink{}, *mPipeline{};
        GMainLoop* mMainLoop{};

        std::thread mMainLoopThread, mStreamSinkThread;

        LoopProfiler mGrabThreadProfiler{get_logger()};

    public:
        UsbCamera();

        ~UsbCamera() override;

        auto pullSampleLoop() -> void;
    };

} // namespace mrover
