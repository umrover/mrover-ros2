#include "key_detector.hpp"
#include <functional>

namespace mrover {

    KeyDetectorBase::KeyDetectorBase(rclcpp::NodeOptions const& options) : rclcpp::Node(NODE_NAME, options), mLoopProfiler{get_logger()} {
        std::string keyDetectionModelName;
        std::string textCoordsModelName;

        std::vector<ParameterWrapper> params{
                {"camera_frame", mCameraFrame, "zed_left_camera_frame"},
                {"world_frame", mWorldFrame, "map"},
                {"increment_weight", mObjIncrementWeight, 2},
                {"decrement_weight", mObjDecrementWeight, 1},
                {"hitcount_threshold", mObjHitThreshold, 5},
                {"hitcount_max", mObjMaxHitcount, 10},
                {"key_detection_model_name", keyDetectionModelName, "yolo-key-detection"},
                {"text_coords_model_name", textCoordsModelName, "magic"},
                {"yolo_model_score_threshold", mModelScoreThreshold, 0.75},
                {"yolo_model_nms_threshold", mModelNMSThreshold, 0.5},
                {"object_detector_debug", mDebug, true}};

        ParameterWrapper::declareParameters(this, params);

        // All of these variables will be invalidated after calling this function

        std::filesystem::path packagePath = std::filesystem::path{ament_index_cpp::get_package_prefix("mrover")} / ".." / ".." / "src" / "mrover";

        // Initialize TensorRT Inference Object and Get Important Output Information
        mKeyDetectionTensorRT = TensortRT{keyDetectionModelName, packagePath.string()};
        mTextCoordsTensorRT = TensortRT{textCoordsModelName, packagePath.string()};

        using namespace std::placeholders;

        mKeyDetectionModel = Model(keyDetectionModelName, {0}, {"key"}, mKeyDetectionTensorRT.getInputTensorSize(), mKeyDetectionTensorRT.getOutputTensorSize(), [](Model const& model, cv::Mat& rgbImage, cv::Mat& blobSizedImage, cv::Mat& blob) { preprocessYOLOv8Input(model, rgbImage, blobSizedImage, blob); }, [this](Model const& model, cv::Mat& output, std::vector<Detection>& detections) { parseYOLOv8Output(model, output, detections); });

        mTextCoordModel = Model(textCoordsModelName, {}, {}, mKeyDetectionTensorRT.getInputTensorSize(), mKeyDetectionTensorRT.getOutputTensorSize(), [](Model const& model, cv::Mat& rgbImage, cv::Mat& blobSizedImage, cv::Mat& blob) { preprocessYOLOv8Input(model, rgbImage, blobSizedImage, blob); }, [this](Model const& model, cv::Mat& output, std::vector<Detection>& detections) { parseYOLOv8Output(model, output, detections); });

        RCLCPP_INFO_STREAM(get_logger(), std::format("Object detector initialized with model: {} and thresholds: {} and {}", mKeyDetectionModel.modelName, mModelScoreThreshold, mModelNMSThreshold));
    }

    ImageKeyDetector::ImageKeyDetector(rclcpp::NodeOptions const& options) : KeyDetectorBase(options) {
        RCLCPP_INFO_STREAM(get_logger(), "Creating Image Object Detector...");

        std::vector<ParameterWrapper> params{
                {"long_range_camera/fov", mCameraHorizontalFov, 80.0}};

        ParameterWrapper::declareParameters(this, params);

        mDebugImgPub = create_publisher<sensor_msgs::msg::Image>("/long_range_object_detector/debug_img", 1);

        mSensorSub = create_subscription<sensor_msgs::msg::Image>("/long_range_cam/image", 1, [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
            ImageKeyDetector::imageCallback(msg);
        });

        mTargetsPub = create_publisher<mrover::msg::ImageTargets>("objects", 1);
    }
} // namespace mrover


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::ImageKeyDetector)
