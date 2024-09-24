#pragma once

#include "pch.hpp"

namespace mrover {

    class ObjectDetectorBase : public rclcpp::Node {

    protected:
        enum MODEL_TYPE {
            YOLOv8 = 0,
        };

        static constexpr char const* NODE_NAME = "object_detector";

        std::unique_ptr<tf2_ros::Buffer> mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        std::shared_ptr<tf2_ros::TransformListener> mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);

        // TF Frames
        std::string mCameraFrame;
        std::string mWorldFrame;

        // Model Member Variables
        MODEL_TYPE mModelType;
        std::string mModelName;
        cv::Mat mRgbImage, mImageBlob;
        std::vector<int64_t> mInputTensorSize;

        LoopProfiler mLoopProfiler;

        TensortRT mTensorRT;

        sensor_msgs::msg::Image mDetectionsImageMessage;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mDebugImgPub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mSensorSub;

        // TODO(quintin): Do not hard code exactly two classes
        std::vector<int> mObjectHitCounts{0, 0};
        std::vector<std::string> mClasses{"bottle", "hammer"};

        int mObjIncrementWeight{};
        int mObjDecrementWeight{};
        int mObjHitThreshold{};
        int mObjMaxHitcount{};
        float mModelScoreThreshold{};
        float mModelNMSThreshold{};

        auto spiralSearchForValidPoint(sensor_msgs::msg::PointCloud2::UniquePtr const& cloudPtr,
                                       std::size_t u, std::size_t v,
                                       std::size_t width, std::size_t height) const -> std::optional<SE3d>;

        auto updateHitsObject(sensor_msgs::msg::PointCloud2::UniquePtr const& msg,
                              std::span<Detection const> detections,
                              cv::Size const& imageSize = {640, 640}) -> void;

        auto publishDetectedObjects(cv::InputArray image) -> void;

        static auto drawDetectionBoxes(cv::InputOutputArray image, std::span<Detection const> detections) -> void;

        auto static parseYOLOv8Output(cv::Mat& output,
                                      std::vector<Detection>& detections,
                                      std::vector<std::string> const& classes,
                                      float modelScoreThreshold = 0.75,
                                      float modelNMSThreshold = 0.5) -> void;

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
