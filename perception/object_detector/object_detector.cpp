#include "object_detector.hpp"
#include <functional>

namespace mrover {

    ObjectDetectorBase::ObjectDetectorBase(rclcpp::NodeOptions const& options) : rclcpp::Node(NODE_NAME, options), mLoopProfiler{get_logger()} {
        std::string bottleModelName;
        std::string malletModelName;
        std::string pickModelName;

        std::vector<ParameterWrapper> params{
                {"camera_frame", mCameraFrame, "zed_left_camera_frame"},
                {"world_frame", mWorldFrame, "map"},
                {"increment_weight", mObjIncrementWeight, 2},
                {"decrement_weight", mObjDecrementWeight, 1},
                {"hitcount_threshold", mObjHitThreshold, 5},
                {"hitcount_max", mObjMaxHitcount, 10},
                {"bottle_model_name", bottleModelName, "bottle"},
                {"mallet_model_name", malletModelName, "mallet"},
                {"pick_model_name", pickModelName, "pick"},
                {"model_score_threshold", mModelScoreThreshold, 0.75},
                {"model_nms_threshold", mModelNMSThreshold, 0.5},
                {"object_detector_debug", mDebug, true}};

        ParameterWrapper::declareParameters(this, params);

        // All of these variables will be invalidated after calling this function

        std::filesystem::path packagePath = std::filesystem::path{ament_index_cpp::get_package_prefix("mrover")} / ".." / ".." / "src" / "mrover";

        RCLCPP_INFO_STREAM(get_logger(), "Opening Model " << bottleModelName);
        RCLCPP_INFO_STREAM(get_logger(), "Opening Model " << malletModelName);
        RCLCPP_INFO_STREAM(get_logger(), "Opening Model " << pickModelName);

        RCLCPP_INFO_STREAM(get_logger(), "Found package path " << packagePath);

        // Initialize TensorRT Inference objects for each model
        mBottleTensorRT = TensortRT{bottleModelName, packagePath.string()};
        mMalletTensorRT = TensortRT{malletModelName, packagePath.string()};
        mPickTensorRT = TensortRT{pickModelName, packagePath.string()};

        // initialize model data structure for each object

        mBottleModel = Model(bottleModelName, {0}, {"bottle"}, {cv::Scalar{0, 4, 227}}, mBottleTensorRT.getInputTensorSize(), mBottleTensorRT.getOutputTensorSize(), [](Model const& model, cv::Mat& rgbImage, cv::Mat& blobSizedImage, cv::Mat& blob) { preprocessYOLOv8Input(model, rgbImage, blobSizedImage, blob); }, [this](Model const& model, cv::Mat& output, std::vector<Detection>& detections) { parseYOLOv8Output(model, output, detections); });
        mMalletModel = Model(malletModelName, {0}, {"mallet"}, {cv::Scalar{232, 115, 5}}, mMalletTensorRT.getInputTensorSize(), mPickTensorRT.getOutputTensorSize(), [](Model const& model, cv::Mat& rgbImage, cv::Mat& blobSizedImage, cv::Mat& blob) { preprocessYOLOv8Input(model, rgbImage, blobSizedImage, blob); }, [this](Model const& model, cv::Mat& output, std::vector<Detection>& detections) { parseYOLOv8Output(model, output, detections); });
        mPickModel = Model(pickModelName, {0}, {"pick"}, {cv::Scalar{5, 225, 5}}, mPickTensorRT.getInputTensorSize(), mMalletTensorRT.getOutputTensorSize(), [](Model const& model, cv::Mat& rgbImage, cv::Mat& blobSizedImage, cv::Mat& blob) { preprocessYOLOv8Input(model, rgbImage, blobSizedImage, blob); }, [this](Model const& model, cv::Mat& output, std::vector<Detection>& detections) { parseYOLOv8Output(model, output, detections); });

        RCLCPP_INFO_STREAM(get_logger(), std::format("Object detector initialized with models: {}, {}, {} and thresholds: {} and {}",  mBottleModel.modelName, mMalletModel.modelName, mPickModel.modelName, mModelScoreThreshold, mModelNMSThreshold));
    }

    StereoObjectDetector::StereoObjectDetector(rclcpp::NodeOptions const& options) : ObjectDetectorBase(options) {
        RCLCPP_INFO_STREAM(get_logger(), "Creating Stereo Object Detector...");

        mDebugImgPub = create_publisher<sensor_msgs::msg::Image>("/stereo_object_detector/annotated_img", 1);

        mSensorSub = create_subscription<sensor_msgs::msg::PointCloud2>("/zed/left/points", 1, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) {
            StereoObjectDetector::pointCloudCallback(msg);
        });

        mServer = create_service<mrover::srv::ToggleObjectDetector>("toggle_stereo_object_detector", [this](mrover::srv::ToggleObjectDetector::Request::ConstSharedPtr request, mrover::srv::ToggleObjectDetector::Response::SharedPtr response) {
            toggleMode(request, response);
        });
    }

    ImageObjectDetector::ImageObjectDetector(rclcpp::NodeOptions const& options) : ObjectDetectorBase(options) {
        RCLCPP_INFO_STREAM(get_logger(), "Creating Image Object Detector...");

        std::vector<ParameterWrapper> params{
                {"long_range_camera/fov", mCameraHorizontalFov, 10.0}};

        ParameterWrapper::declareParameters(this, params);

        mDebugImgPub = create_publisher<sensor_msgs::msg::Image>("/long_range_object_detector/annotated_img", 1);

        mSensorSub = create_subscription<sensor_msgs::msg::Image>("/long_range_cam/image", 1, [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
            ImageObjectDetector::imageCallback(msg);
        });

        mTargetsPub = create_publisher<mrover::msg::ImageTargets>("objects", 1);

        mServer = create_service<mrover::srv::ToggleObjectDetector>("toggle_image_object_detector", [this](mrover::srv::ToggleObjectDetector::Request::ConstSharedPtr request, mrover::srv::ToggleObjectDetector::Response::SharedPtr response) {
            toggleMode(request, response);
        });
    }
} // namespace mrover


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::StereoObjectDetector)
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::ImageObjectDetector)
