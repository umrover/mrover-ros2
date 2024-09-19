#include "object_detector.hpp"

namespace mrover {

    ObjectDetectorBase::ObjectDetectorBase() : rclcpp::Node(NODE_NAME), mLoopProfiler{get_logger()} {

        std::vector<ParameterWrapper> params{
                {"camera_frame", mCameraFrame, "zed_left_camera_frame"},
                {"world_frame", mWorldFrame, "map"},
                {"increment_weight", mObjIncrementWeight, 2},
                {"decrement_weight", mObjDecrementWeight, 1},
                {"hitcount_threshold", mObjHitThreshold, 5},
                {"hitcount_max", mObjMaxHitcount, 10},
                {"model_name", mModelName, "Large-Dataset"},
                {"model_score_threshold", mModelScoreThreshold, 0.75},
                {"model_nms_threshold", mModelNmsThreshold, 0.5}
		};

        ParameterWrapper::declareParameters(this, params);

        std::string packagePath{"/home/john/ros2_ws/src/mrover"};

        RCLCPP_INFO_STREAM(get_logger(), "Opening Model " << mModelName);

        mTensorRT = TensortRT{mModelName, packagePath};

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
