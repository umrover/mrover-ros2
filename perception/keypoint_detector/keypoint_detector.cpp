#include "keypoint_detector.hpp"

namespace mrover {

    KeypointDetectorBase::KeypointDetectorBase(std::string const& name, rclcpp::NodeOptions const& options) : rclcpp::Node(name, options), mLoopProfiler{get_logger()} {
        std::string keypointModelName;

        int keypointThreshold;

        std::vector<ParameterWrapper> params{
                {"camera_frame", mCameraFrame, "zed_left_camera_frame"},
                {"world_frame", mWorldFrame, "map"},
                {"increment_weight", mObjIncrementWeight, 2},
                {"decrement_weight", mObjDecrementWeight, 1},
                {"hitcount_max", mObjMaxHitcount, 10},
                {"keypoint_model_name", keypointModelName, "pick-keypoint"},
                {"keypoint_threshold", keypointThreshold, 5},
                {"keypoint_score_threshold", mKeypointScoreThreshold, 0.75},
                {"model_score_threshold", mModelScoreThreshold, 0.75},
                {"model_nms_threshold", mModelNMSThreshold, 0.5},
                {"object_detector_debug", mDebug, true}};

        ParameterWrapper::declareParameters(this, params);

        std::filesystem::path packagePath = std::filesystem::path{ament_index_cpp::get_package_prefix("mrover")} / ".." / ".." / "src" / "mrover";

        RCLCPP_INFO_STREAM(get_logger(), "Opening Model " << keypointModelName);

        RCLCPP_INFO_STREAM(get_logger(), "Found package path " << packagePath);

        mModelScoreThreshold = 0.75;

        RCLCPP_INFO_STREAM(get_logger(), "Keypoint Threshold " << mKeypointScoreThreshold);

        mKeypointTensorRT = TensortRT{keypointModelName, packagePath.string()};
        mKeypointModel = Model(keypointModelName, {0}, keypointThreshold, {"keypoint"}, {cv::Scalar{5, 225, 5}}, mKeypointTensorRT.getInputTensorSize(), mKeypointTensorRT.getOutputTensorSize(), [](Model const& model, cv::Mat& rgbImage, cv::Mat& blobSizedImage, cv::Mat& blob) { preprocessYOLOv26Input(model, rgbImage, blobSizedImage, blob); }, [this](Model const& model, cv::Mat& output, std::vector<Detection>& detections) { parseYOLOv26Output(model, output, detections); });

        RCLCPP_INFO_STREAM(get_logger(), std::format("Keypoint detector initialized with model: {} and thresholds: {} and {}", mKeypointModel.modelName, mModelScoreThreshold, mModelNMSThreshold));
    }

    ImageKeypointDetector::ImageKeypointDetector(rclcpp::NodeOptions const& options) : KeypointDetectorBase("image_keypoint_detector", options) {
        RCLCPP_INFO_STREAM(get_logger(), "Creating Image Keypoint Detector...");

        std::vector<ParameterWrapper> params{
                {"auton_arm_camera/fov", mCameraHorizontalFov, 10.0}, //not real
                {"detection_distance_threshold", mDistanceDetectionThreshold, std::numeric_limits<float>::infinity()}};

        ParameterWrapper::declareParameters(this, params);

        mDebugImgPub = create_publisher<sensor_msgs::msg::Image>("/auton_arm_keypoint_detector/annotated_img", 1);

        mSensorSub = create_subscription<sensor_msgs::msg::Image>("/auton_arm_cam/image", 1, [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
            ImageKeypointDetector::imageCallback(msg);
        });

        mTargetsPub = create_publisher<mrover::msg::ImageTargets>("objects", 1);

        mServer = create_service<mrover::srv::ToggleObjectDetector>("toggle_image_keypoint_detector", [this](mrover::srv::ToggleObjectDetector::Request::ConstSharedPtr request, mrover::srv::ToggleObjectDetector::Response::SharedPtr response) {
            toggleMode(request, response);
        });
    }
} // namespace mrover


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::ImageKeypointDetector)