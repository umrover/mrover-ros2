#pragma once

#include "pch.hpp"

namespace mrover {

    class UsbCamera final : public rclcpp::Node {

        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mCamInfoPub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mImgPub;

        int mWidth{}, mHeight{};

        GstElement *mStreamSink{}, *mPipeline{};
        GMainLoop* mMainLoop{};

        std::thread mMainLoopThread, mStreamSinkThread;

    public:
        explicit UsbCamera(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        ~UsbCamera() override;

        auto pullSampleLoop() -> void;
    };

} // namespace mrover
