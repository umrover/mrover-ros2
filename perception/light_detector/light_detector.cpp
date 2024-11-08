#include "light_detector.hpp"
#include <opencv2/core/matx.hpp>
#include <rclcpp/node.hpp>

namespace mrover{

	LightDetector::LightDetector(rclcpp::NodeOptions const& options) : rclcpp::Node(NODE_NAME, options) {
		RCLCPP_INFO_STREAM(get_logger(),"Light Detector Initializing");

		int upperBoundH = 0;
		int upperBoundS = 0;
		int upperBoundV = 0;
		int lowerBoundH = 0;
		int lowerBoundS = 0;
		int lowerBoundV = 0;

		std::vector<ParameterWrapper> params{
                {"camera_frame", mCameraFrame, "zed_left_camera_frame"},
                {"world_frame", mWorldFrame, "map"},
                {"light_detector/spiral_search_radius", SPIRAL_SEARCH_DIM, 50},
                {"light_detector/immediate_light_range", mImmediateLightRange, 5},
                {"light_detector/hit_increase", mHitIncrease, 5},
                {"light_detector/hit_decrease", mHitDecrease, 2},
                {"light_detector/hit_max", mHitMax, 100},
                {"light_detector/upper_bound_h", upperBoundH, 0},
				{"light_detector/upper_bound_s", upperBoundS, 0},
				{"light_detector/upper_bound_v", upperBoundV, 0},
				{"light_detector/lower_bound_h", lowerBoundH, 0},
				{"light_detector/lower_bound_s", lowerBoundS, 0},
				{"light_detector/lower_bound_v", lowerBoundV, 0}};

        ParameterWrapper::declareParameters(this, params);

		mUpperBound = cv::Vec3d(upperBoundH, upperBoundS, upperBoundV);
		mLowerBound = cv::Vec3d(lowerBoundH, lowerBoundS, lowerBoundV);

		// TODO: fix this next time! check wiki for what the message type should be
		imgSub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/camera/left/points", 1, &LightDetector::imageCallback);
		imgPub = this->create_publisher<sensor_msgs::msg::Image>("/light_detector/img", 1);
		pointPub = this->create_publisher<geometry_msgs::msg::Vector3>("/light_detector/points", 1);
	}

} // mrover