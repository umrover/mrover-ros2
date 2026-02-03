#pragma once

#include "pch.hpp"

namespace mrover {

    class ObjectDetectorBase : public rclcpp::Node {
    protected:
        struct Detection {
            int classId{};
            std::string className;
            float confidence{};
            cv::Rect box;

            Detection(int _classId, std::string _className, float _confidence, cv::Rect _box) : classId{_classId}, className{_className}, confidence{_confidence}, box{_box} {}
        };

        struct Model {
            std::string modelName;

            std::vector<int> objectHitCounts;

            std::vector<std::string> classes;

            std::vector<int64_t> inputTensorSize;

            std::vector<int64_t> outputTensorSize;

            // Converts an rgb image to a blob
            std::function<void(Model const&, cv::Mat&, cv::Mat&, cv::Mat&)> rgbImageToBlob;

            // Converts an output tensor into a vector of detections
            std::function<void(Model const&, cv::Mat&, std::vector<Detection>&)> outputTensorToDetections;

            Model() = default;

            Model(std::string _modelName, std::vector<int> _objectHitCounts, std::vector<std::string> _classes, std::vector<int64_t> _inputTensorSize, std::vector<int64_t> _outputTensorSize, std::function<void(Model const&, cv::Mat&, cv::Mat&, cv::Mat&)> _rbgImageToBlob, std::function<void(Model const&, cv::Mat&, std::vector<Detection>&)> _outputTensorToDetections) : modelName{std::move(_modelName)}, objectHitCounts(std::move(_objectHitCounts)), classes(std::move(_classes)), inputTensorSize(std::move(_inputTensorSize)), outputTensorSize(std::move(_outputTensorSize)), rgbImageToBlob{std::move(_rbgImageToBlob)}, outputTensorToDetections{std::move(_outputTensorToDetections)} {}
        };


        static constexpr char const* NODE_NAME = "object_detector";

        std::unique_ptr<tf2_ros::Buffer> mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        std::shared_ptr<tf2_ros::TransformListener> mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);

        Model mModel;

        std::string mCameraFrame;
        std::string mWorldFrame;
        std::string mImageSubTopic;

        cv::Mat mRgbImage, mImageBlob;

        LoopProfiler mLoopProfiler;

        TensortRT mTensorRT;

        sensor_msgs::msg::Image mDetectionsImageMessage;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mDebugImgPub;
        rclcpp::Publisher<mrover::msg::ObjectBoundingBoxes>::SharedPtr mBoxesPub;

        int mObjIncrementWeight{};
        int mObjDecrementWeight{};
        int mObjHitThreshold{};
        int mObjMaxHitcount{};
        float mModelScoreThreshold{};
        float mModelNMSThreshold{};
        bool mDebug{};

        auto spiralSearchForValidPoint(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& cloudPtr,
                                       std::size_t u, std::size_t v,
                                       std::size_t width, std::size_t height) const -> std::optional<SE3d>;

        auto updateHitsObject(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg,
                              std::span<Detection const> detections,
                              cv::Size const& imageSize = {640, 640}) -> void;

        auto publishDetectedObjects(cv::InputArray const& image) -> void;

        static auto drawDetectionBoxes(cv::InputOutputArray& image, std::span<Detection const> detections) -> void;

        auto parseYOLOv8Output(Model const& model,
                               cv::Mat& output,
                               std::vector<Detection>& detections) const -> void;

        static auto preprocessYOLOv8Input(Model const& model, cv::Mat const& rgbImage, cv::Mat& blobSizedImage, cv::Mat& blob) -> void;

        auto resizeBoundingBoxes(cv::Size const& outputSpace, std::vector<Detection>& detections) const -> void;

    public:
        explicit ObjectDetectorBase(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        ~ObjectDetectorBase() override = default;
    };

    class StereoObjectDetector final : public ObjectDetectorBase {
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mSensorSub;

    public:
        explicit StereoObjectDetector(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        static auto convertPointCloudToRGB(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg, cv::Mat const& image) -> void;

        auto pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void;
    };

    class ImageObjectDetector final : public ObjectDetectorBase {
    private:
        rclcpp::Publisher<mrover::msg::ImageTargets>::SharedPtr mTargetsPub;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mSensorSub;

        float mCameraHorizontalFov{};

    public:
        explicit ImageObjectDetector(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        auto getObjectBearing(cv::InputArray const& image, cv::Rect const& box) const -> float;

        auto imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;
    };
} // namespace mrover
