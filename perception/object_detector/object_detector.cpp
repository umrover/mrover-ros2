#include "object_detector.hpp"

namespace mrover {

    ObjectDetectorBase::ObjectDetectorBase() : rclcpp::Node(NODE_NAME), mLoopProfiler{get_logger()} {

        auto paramSub = std::make_shared<rclcpp::ParameterEventHandler>(this);

        std::vector<ParameterWrapper> params{
                {"camera_frame", mCameraFrame},
                {"world_frame", mWorldFrame},
                {"increment_weight", mObjIncrementWeight},
                {"decrement_weight", mObjDecrementWeight},
                {"hitcount_threshold", mObjHitThreshold},
                {"hitcount_max", mObjMaxHitcount},
                {"model_name", mModelName},
                {"model_score_threshold", mModelScoreThreshold},
                {"model_nms_threshold", mModelNmsThreshold}};

        ParameterWrapper::declareParameters(this, paramSub, params);

        std::string packagePath{"/home/john/ros2_ws/src/mrover"};

        RCLCPP_INFO_STREAM(get_logger(), "Opening Model " << mModelName);

        mLearning = Learning{mModelName, packagePath};

        mDebugImgPub = create_publisher<sensor_msgs::msg::Image>("object_detector/debug_img", 1);

        RCLCPP_INFO_STREAM(get_logger(), std::format("Object detector initialized with model: {} and thresholds: {} and {}", mModelName, mModelScoreThreshold, mModelNmsThreshold));
    }

    StereoObjectDetector::StereoObjectDetector() {
        mSensorSub = create_subscription<sensor_msgs::msg::PointCloud2>("/camera/left/points", 1, [this](sensor_msgs::msg::PointCloud2::UniquePtr const& msg) {
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
