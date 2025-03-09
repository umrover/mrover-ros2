#pragma once

#include "pch.hpp"

namespace mrover {

    class KeyDetectorBase : public rclcpp::Node {
    protected:
        struct Detection {
            int classId{};
            std::string className;
            float confidence{};
            cv::Rect box;

            Detection(int _classId, std::string _className, float _confidence, cv::Rect _box) : classId{_classId}, className{_className}, confidence{_confidence}, box{_box} {}
        };

        struct Model;

        // model processing functions
        using model_preprocess_t = std::function<void(Model const&, cv::Mat const&, cv::Mat&)>;
        using model_postprocess_t = std::function<void(Model const&, cv::Mat&)>;

        struct Model {
            // TODO: remove
            rclcpp::Node* ptr;

            std::string modelName;

            std::vector<int> objectHitCounts;

            std::vector<std::string> classes;

            std::vector<int64_t> inputTensorSize;

            std::vector<int64_t> outputTensorSize;

            // Converts an rgb image to a blob
            model_preprocess_t preprocess;

            // Converts an output tensor into a vector of detections
            model_postprocess_t postprocess;

            Model() = default;

            Model(rclcpp::Node* _ptr, std::string _modelName, std::vector<int> _objectHitCounts, std::vector<std::string> _classes, std::vector<int64_t> _inputTensorSize, std::vector<int64_t> _outputTensorSize, model_preprocess_t _preprocess, model_postprocess_t _postprocess) : ptr{_ptr}, modelName{std::move(_modelName)}, objectHitCounts(std::move(_objectHitCounts)), classes(std::move(_classes)), inputTensorSize(std::move(_inputTensorSize)), outputTensorSize(std::move(_outputTensorSize)), preprocess{std::move(_preprocess)}, postprocess{std::move(_postprocess)} {}
        };


        static constexpr char const* NODE_NAME = "image_key_detector";

        std::unique_ptr<tf2_ros::Buffer> mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        std::shared_ptr<tf2_ros::TransformListener> mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);

        Model mKeyDetectionModel;
        Model mTextCoordModel;

        std::string mCameraFrame;
        std::string mWorldFrame;

        cv::Mat mRgbImage, mImageBlob, mTextCoordsBlob;

        LoopProfiler mLoopProfiler;

        TensortRT mKeyDetectionTensorRT;
        TensortRT mTextCoordsTensorRT;

        sensor_msgs::msg::Image mDetectionsImageMessage;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mDebugImgPub;

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

        auto drawDetectionBoxes(cv::InputOutputArray& image, std::span<Detection const> detections) const -> void;

        auto parseYOLOv8Output(Model const& model, cv::Mat& output, std::vector<Detection>& detections) const -> void;

        // Pre and Post Process for YOLO
        static auto preprocessYOLOv8Input(Model const& model, cv::Mat const& input, cv::Mat& output) -> void;

        static auto postprocessYOLOv8Output(Model const& model, cv::Mat& output) -> void;

        // Pre and Post Process for Text Coords
        static auto preprocessTextCoordsInput(Model const& model, cv::Mat const& input, cv::Mat& output) -> void;

        static auto postprocessTextCoordsOutput(Model const& model, cv::Mat& output) -> void;

    public:
        explicit KeyDetectorBase(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        ~KeyDetectorBase() override = default;
    };

    class ImageKeyDetector final : public KeyDetectorBase {
    private:
        rclcpp::Publisher<mrover::msg::ImageTargets>::SharedPtr mTargetsPub;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mSensorSub;

        float mCameraHorizontalFov{};

    public:
        explicit ImageKeyDetector(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        auto getObjectBearing(std::size_t imageWidth, cv::Rect const& box) const -> float;

        auto imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;
    };
} // namespace mrover
