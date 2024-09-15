#pragma once

#include "pch.hpp"

namespace mrover {

    class ObjectDetectorBase : public rclcpp::Node {

    protected:
        static constexpr char const* NODE_NAME = "object_detector";

        std::unique_ptr<tf2_ros::Buffer> mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        std::shared_ptr<tf2_ros::TransformListener> mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);

        std::string mCameraFrame;
        std::string mWorldFrame;

        std::string mModelName;

        LoopProfiler mLoopProfiler;

        Learning mLearning;

        cv::Mat mRgbImage, mImageBlob;
        sensor_msgs::msg::Image mDetectionsImageMessage;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mDebugImgPub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mSensorSub;

        // TODO(quintin): Do not hard code exactly two classes
        std::vector<int> mObjectHitCounts{0, 0};

        int mObjIncrementWeight{};
        int mObjDecrementWeight{};
        int mObjHitThreshold{};
        int mObjMaxHitcount{};
        float mModelScoreThreshold{};
        float mModelNmsThreshold{};

        auto spiralSearchForValidPoint(sensor_msgs::msg::PointCloud2::UniquePtr const& cloudPtr,
                                       std::size_t u, std::size_t v,
                                       std::size_t width, std::size_t height) const -> std::optional<SE3d>;

        auto updateHitsObject(sensor_msgs::msg::PointCloud2::UniquePtr const& msg,
                              std::span<Detection const> detections,
                              cv::Size const& imageSize = {640, 640}) -> void;

        auto publishDetectedObjects(cv::InputArray image) -> void;

        static auto drawDetectionBoxes(cv::InputOutputArray image, std::span<Detection const> detections) -> void;

    public:
        explicit ObjectDetectorBase();

        ~ObjectDetectorBase() override = default;
    };

    class StereoObjectDetector final : public ObjectDetectorBase {
    public:
        explicit StereoObjectDetector();

        static auto convertPointCloudToRGB(sensor_msgs::msg::PointCloud2::UniquePtr const& msg, cv::Mat const& image) -> void;

        auto pointCloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr const& msg) -> void;
    };
} // namespace mrover
