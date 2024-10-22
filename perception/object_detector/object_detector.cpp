#include "object_detector.hpp"

namespace mrover {

    ObjectDetectorBase::ObjectDetectorBase(rclcpp::NodeOptions const& options) : rclcpp::Node(NODE_NAME, options), mLoopProfiler{get_logger()} {
        std::string modelName;
        float modelScoreThreshold{};
        float modelNMSThreshold{};

        std::vector<ParameterWrapper> params{
                {"camera_frame", mCameraFrame, "zed_left_camera_frame"},
                {"world_frame", mWorldFrame, "map"},
                {"increment_weight", mObjIncrementWeight, 2},
                {"decrement_weight", mObjDecrementWeight, 1},
                {"hitcount_threshold", mObjHitThreshold, 5},
                {"hitcount_max", mObjMaxHitcount, 10},
                {"model_name", modelName, "Large-Dataset"},
                {"model_score_threshold", modelScoreThreshold, 0.75},
                {"model_nms_threshold", modelNMSThreshold, 0.5}};

        ParameterWrapper::declareParameters(this, params);

        // All of these variables will be invalidated after calling this function

        std::filesystem::path packagePath = std::filesystem::path{ament_index_cpp::get_package_prefix("mrover")} / ".." / ".." / "src" / "mrover";

        RCLCPP_INFO_STREAM(get_logger(), "Opening Model " << modelName);

        RCLCPP_INFO_STREAM(get_logger(), "Found package path " << packagePath);

        // Initialize TensorRT Inference Object and Get Important Output Information
        mTensorRT = TensortRT{modelName, packagePath.string()};

        mModel = Model(modelName, {0, 0}, {"bottle", "hammer"}, mTensorRT.getInputTensorSize(), mTensorRT.getOutputTensorSize(), {modelScoreThreshold, modelNMSThreshold}, ObjectDetectorBase::preprocessYOLOv8Input, ObjectDetectorBase::parseYOLOv8Output);

        RCLCPP_INFO_STREAM(get_logger(), std::format("Object detector initialized with model: {} and thresholds: {} and {}", mModel.modelName, modelScoreThreshold, modelNMSThreshold));
    }

    StereoObjectDetector::StereoObjectDetector(rclcpp::NodeOptions const& options) : ObjectDetectorBase(options) {
        RCLCPP_INFO_STREAM(get_logger(), "Creating Stereo Object Detector...");

        mDebugImgPub = create_publisher<sensor_msgs::msg::Image>("/stereo_object_detector/debug_img", 1);

        mSensorSub = create_subscription<sensor_msgs::msg::PointCloud2>("/zed/left/points", 1, [this](sensor_msgs::msg::PointCloud2::UniquePtr const& msg) {
            StereoObjectDetector::pointCloudCallback(msg);
        });
    }

    ImageObjectDetector::ImageObjectDetector(rclcpp::NodeOptions const& options) : ObjectDetectorBase(options) {
        RCLCPP_INFO_STREAM(get_logger(), "Creating Image Object Detector...");

        std::vector<ParameterWrapper> params{
                {"long_range_camera/fov", mCameraHorizontalFov, 80.0}};

        ParameterWrapper::declareParameters(this, params);

        mDebugImgPub = create_publisher<sensor_msgs::msg::Image>("/long_range_object_detector/debug_img", 1);

        mSensorSub = create_subscription<sensor_msgs::msg::Image>("/usb_camera/image", 1, [this](sensor_msgs::msg::Image::UniquePtr const& msg) {
            ImageObjectDetector::imageCallback(msg);
        });

        mTargetsPub = create_publisher<mrover::msg::ImageTargets>("/long_range_camera/objects", 1);
    }
} // namespace mrover


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::StereoObjectDetector)
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::ImageObjectDetector)
