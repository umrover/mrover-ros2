#include "object_detector.hpp"

namespace mrover {

    ObjectDetectorBase::ObjectDetectorBase() : rclcpp::Node(NODE_NAME), mLoopProfiler{get_logger()} {
        RCLCPP_INFO_STREAM(get_logger(), "Opening Model");

        std::string modelName;
        float modelScoreThreshold{};
        float modelNMSThreshold{};

        RCLCPP_INFO_STREAM(get_logger(), "Opening Model");
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

        RCLCPP_INFO_STREAM(get_logger(), "Opening Model");
        ParameterWrapper::declareParameters(this, params);

        // All of these variables will be invalidated after calling this function

        std::filesystem::path packagePath = std::filesystem::path{ament_index_cpp::get_package_prefix("mrover")} / ".." / ".." / "src" / "mrover";

        RCLCPP_INFO_STREAM(get_logger(), "Opening Model");
        RCLCPP_INFO_STREAM(get_logger(), "Opening Model " << modelName);

        RCLCPP_INFO_STREAM(get_logger(), "Opening Model");
        RCLCPP_INFO_STREAM(get_logger(), "Found package path " << packagePath);

        // Initialize TensorRT Inference Object and Get Important Output Information
        mTensorRT = TensortRT{modelName, packagePath.string()};
        RCLCPP_INFO_STREAM(get_logger(), "Opening Model");

        mModel = Model(modelName, {0, 0}, {"bottle", "hammer"}, mTensorRT.getInputTensorSize(), mTensorRT.getOutputTensorSize(), {modelScoreThreshold, modelNMSThreshold}, ObjectDetectorBase::preprocessYOLOv8Input, ObjectDetectorBase::parseYOLOv8Output);

        RCLCPP_INFO_STREAM(get_logger(), "Opening Model");
        RCLCPP_INFO_STREAM(get_logger(), std::format("Object detector initialized with model: {} and thresholds: {} and {}", mModel.modelName, modelScoreThreshold, modelNMSThreshold));
    }

    StereoObjectDetector::StereoObjectDetector() {
        mDebugImgPub = create_publisher<sensor_msgs::msg::Image>("/stereo_object_detector/debug_img", 1);
        RCLCPP_INFO_STREAM(get_logger(), "Opening Model");

        mSensorSub = create_subscription<sensor_msgs::msg::PointCloud2>("/camera/left/points", 1, [this](sensor_msgs::msg::PointCloud2::UniquePtr const& msg) {
            StereoObjectDetector::pointCloudCallback(msg);
        });
        RCLCPP_INFO_STREAM(get_logger(), "Opening Model");
    }

    ImageObjectDetector::ImageObjectDetector() {
        std::vector<ParameterWrapper> params{
                {"long_range_camera/fov", mCameraHorizontalFov, 80.0}};
        RCLCPP_INFO_STREAM(get_logger(), "Opening Model");

        ParameterWrapper::declareParameters(this, params);

        RCLCPP_INFO_STREAM(get_logger(), "Opening Model");
        mDebugImgPub = create_publisher<sensor_msgs::msg::Image>("/long_range_object_detector/debug_img", 1);

        mSensorSub = create_subscription<sensor_msgs::msg::Image>("/usb_camera/image", 1, [this](sensor_msgs::msg::Image::UniquePtr const& msg) {
            ImageObjectDetector::imageCallback(msg);
        });
        RCLCPP_INFO_STREAM(get_logger(), "Opening Model");

        mTargetsPub = create_publisher<mrover::msg::ImageTargets>("/long_range_camera/objects", 1);
        RCLCPP_INFO_STREAM(get_logger(), "Opening Model");
    }
} // namespace mrover


auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

	rclcpp::executors::SingleThreadedExecutor executor;
	//executor.add_node(std::make_shared<mrover::StereoObjectDetector>());
	executor.add_node(std::make_shared<mrover::ImageObjectDetector>());
	executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
