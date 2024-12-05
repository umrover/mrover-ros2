#include "object_detector.hpp"

namespace mrover {

    ObjectDetectorBase::ObjectDetectorBase() : rclcpp::Node(NODE_NAME), mLoopProfiler{get_logger()} {

        std::vector<ParameterWrapper> params{
                {this, "camera_frame", mCameraFrame, "zed_left_camera_frame"},
                {this, "world_frame", mWorldFrame, "map"},
                {this, "increment_weight", mObjIncrementWeight, 2},
                {this, "decrement_weight", mObjDecrementWeight, 1},
                {this, "hitcount_threshold", mObjHitThreshold, 5},
                {this, "hitcount_max", mObjMaxHitcount, 10},
                {this, "model_name", mModelName, "Large-Dataset"},
                {this, "model_score_threshold", mModelScoreThreshold, 0.75},
                {this, "model_nms_threshold", mModelNmsThreshold, 0.5}};

        std::filesystem::path packagePath = std::filesystem::path{ament_index_cpp::get_package_prefix("mrover")} / ".." / ".." / "src" / "mrover";

        RCLCPP_INFO_STREAM(get_logger(), "Opening Model " << mModelName);

        RCLCPP_INFO_STREAM(get_logger(), "Found package path " << packagePath);

        mTensorRT = TensortRT{mModelName, packagePath.string()};

        mDebugImgPub = create_publisher<sensor_msgs::msg::Image>("object_detector/debug_img", 1);

        RCLCPP_INFO_STREAM(get_logger(), std::format("Object detector initialized with model: {} and thresholds: {} and {}", mModelName, mModelScoreThreshold, mModelNmsThreshold));
    }

    StereoObjectDetector::StereoObjectDetector() {
        mSensorSub = create_subscription<sensor_msgs::msg::PointCloud2>("/zed/left/points", 1, [this](sensor_msgs::msg::PointCloud2::UniquePtr const& msg) {
            StereoObjectDetector::pointCloudCallback(msg);
        });
    }
} // namespace mrover


auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::StereoObjectDetector>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
