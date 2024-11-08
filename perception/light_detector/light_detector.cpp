#include "light_detector.hpp"
#include <opencv2/core/matx.hpp>
#include <rclcpp/node.hpp>

namespace mrover{

	auto LightDetector::onInit() -> void{
		RCLCPP_INFO_STREAM(get_logger(),"Light Detector Initializing");

		this->declare_parameter<std::string>("camera_frame", "zed_left_camera_frame");
		mCameraFrame = this->get_parameter("camera_frame").as_string();

		this->declare_parameter<std::string>("world_frame", "map");
		mWorldFrame = this->get_parameter("world_frame").as_string();

		this->declare_parameter<int>("light_detector/spiral_search_radius", 50);
		SPIRAL_SEARCH_DIM = this->get_parameter("light_detector/spiral_search_radius").as_int();
		
		this->declare_parameter<int>("light_detector/immediate_light_range", 5);
		mImmediateLightRange = this->get_parameter("light_detector/immediate_light_range").as_double();
		
		this->declare_parameter<int>("light_detector/hit_increase", 5);
		mHitIncrease = this->get_parameter("light_detector/hit_increase").as_int();
		
		this->declare_parameter<int>("light_detector/hit_decrease", 2);
		mHitDecrease = this->get_parameter("light_detector/hit_decrease").as_int();

		this->declare_parameter<int>("light_detector/hit_max", 100);
		mHitMax = this->get_parameter("light_detector/hit_max").as_int();
		
		this->declare_parameter<int>("light_detector/pub_threshold", 50);
		mPublishThreshold = this->get_parameter("light_detector/pub_threshold").as_int();

		int upperBoundH = 0;
		int upperBoundS = 0;
		int upperBoundV = 0;
		int lowerBoundH = 0;
		int lowerBoundS = 0;
		int lowerBoundV = 0;

		this->declare_parameter<int>("light_detector/upper_bound_h", 0);
		upperBoundH = this->get_parameter("light_detector/upper_bound_h").as_int();

		this->declare_parameter<int>("light_detector/upper_bound_s", 0);
		upperBoundS = this->get_parameter("light_detector/upper_bound_s").as_int();

		this->declare_parameter<int>("light_detector/upper_bound_v", 0);
		upperBoundV = this->get_parameter("light_detector/upper_bound_v").as_int();

		this->declare_parameter<int>("light_detector/lower_bound_h", 0);
		lowerBoundH = this->get_parameter("light_detector/lower_bound_h").as_int();

		this->declare_parameter<int>("light_detector/lower_bound_s", 0);
		lowerBoundS = this->get_parameter("light_detector/lower_bound_s").as_int();

		this->declare_parameter<int>("light_detector/lower_bound_v", 0);
		lowerBoundV = this->get_parameter("light_detector/lower_bound_v").as_int();

		mUpperBound = cv::Vec3d(upperBoundH, upperBoundS, upperBoundV);
		mLowerBound = cv::Vec3d(lowerBoundH, lowerBoundS, lowerBoundV);

		// TODO: fix this next time! check wiki for what the message type should be
		imgSub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/camera/left/points", 1, &LightDetector::imageCallback, this);
		imgPub = this->get_publisher<sensor_msgs::msg::Image>("/light_detector/img", 1);
		pointPub = this->get_publisher<geometry_msgs::msg::Vector3>("/light_detector/points", 1);
	}

	LightDetector::LightDetector() {
        RCLCPP_INFO_STREAM(get_logger(), "Creating Light Detector...");

        std::vector<ParameterWrapper> params{
                {"long_range_camera/fov", mCameraHorizontalFov, 80.0}};

        ParameterWrapper::declareParameters(this, params);

        mDebugImgPub = create_publisher<sensor_msgs::msg::Image>("/long_range_object_detector/debug_img", 1);

        mSensorSub = create_subscription<sensor_msgs::msg::Image>("/usb_camera/image", 1, [this](sensor_msgs::msg::Image::UniquePtr const& msg) {
            ImageObjectDetector::imageCallback(msg);
        });

        mTargetsPub = create_publisher<mrover::msg::ImageTargets>("/long_range_camera/objects", 1);
    }

} // mrover