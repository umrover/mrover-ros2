#include "tag_detector.hpp"

namespace mrover {

    auto threshold(cv::InputArray in, cv::OutputArray out, int windowSize, double constant) -> void {
        CV_Assert(windowSize >= 3);

        if (windowSize % 2 == 0) windowSize++; // Window size must be odd
        cv::adaptiveThreshold(in, out, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, windowSize, constant);
    }

    auto TagDetectorBase::publishThresholdedImage() -> void {
        cvtColor(mBgrImage, mGrayImage, cv::COLOR_BGR2GRAY);

        // Number of window sizes (scales) to apply adaptive thresholding
        int scaleCount = (mDetectorParams->adaptiveThreshWinSizeMax - mDetectorParams->adaptiveThreshWinSizeMin) / mDetectorParams->adaptiveThreshWinSizeStep + 1;

        // For each value in the interval of thresholding window sizes
        for (int scale = 0; scale < scaleCount; ++scale) {
            auto it = mThreshImagePubs.find(scale);
            if (it == mThreshImagePubs.end()) {
                RCLCPP_INFO_STREAM(get_logger(), std::format("Creating new publisher for thresholded scale {}", scale));
                std::tie(it, std::ignore) = mThreshImagePubs.emplace(scale, create_publisher<sensor_msgs::msg::Image>(std::format("tag_detection_threshold_{}", scale), 1));
            }
            auto& [_, publisher] = *it;

            if (publisher->get_subscription_count() == 0) continue;

            int windowSize = mDetectorParams->adaptiveThreshWinSizeMin + scale * mDetectorParams->adaptiveThreshWinSizeStep;
            threshold(mGrayImage, mGrayImage, windowSize, mDetectorParams->adaptiveThreshConstant);

            mThresholdImageMessage.header.stamp = get_clock()->now();
            mThresholdImageMessage.header.frame_id = "zed_left_camera_frame";
            mThresholdImageMessage.height = mGrayImage.rows;
            mThresholdImageMessage.width = mGrayImage.cols;
            mThresholdImageMessage.encoding = sensor_msgs::image_encodings::MONO8;
            mThresholdImageMessage.step = mGrayImage.step;
            mThresholdImageMessage.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
            size_t size = mThresholdImageMessage.step * mThresholdImageMessage.height;
            mThresholdImageMessage.data.resize(size);
            std::memcpy(mThresholdImageMessage.data.data(), mGrayImage.data, size);

            publisher->publish(mThresholdImageMessage);
        }
    }

} // namespace mrover
