#include "light_detector.hpp"
#include "tensorrt.hpp"
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

		std::string modelName;

		std::vector<ParameterWrapper> params{
                {"camera_frame", mCameraFrame, "zed_left_camera_frame"},
                {"world_frame", mWorldFrame, "map"},
                {"light_detector/spiral_search_radius", SPIRAL_SEARCH_DIM, 50},
                {"light_detector/immediate_light_range", mImmediateLightRange, 5},
                {"light_detector/hit_increase", mHitIncrease, 5},
                {"light_detector/hit_decrease", mHitDecrease, 1},
                {"light_detector/hit_max", mHitMax, 100},
				{"light_detector/m_publish_threshold", mPublishThreshold, 50},
                {"light_detector/upper_bound_h", upperBoundH, 0},
				{"light_detector/upper_bound_s", upperBoundS, 0},
				{"light_detector/upper_bound_v", upperBoundV, 0},
				{"light_detector/lower_bound_h", lowerBoundH, 0},
				{"light_detector/lower_bound_s", lowerBoundS, 0},
				{"light_detector/lower_bound_v", lowerBoundV, 0},
				{"light_detector/lmodel_name", modelName, "GreenWaterBottle"},
                {"light_detector/model_score_threshold", mModelScoreThreshold, 0.75},
            	{"light_detector/model_nms_threshold", mModelNMSThreshold, 0.5},
            	{"light_detector/debug", mDebug, true}};


        ParameterWrapper::declareParameters(this, params);
        
		std::filesystem::path packagePath = std::filesystem::path{ament_index_cpp::get_package_prefix("mrover")} / ".." / ".." / "src" / "mrover";

        RCLCPP_INFO_STREAM(get_logger(), "Opening Model " << modelName);

        RCLCPP_INFO_STREAM(get_logger(), "Found package path " << packagePath);

        // Initialize TensorRT Inference Object and Get Important Output Information
        mTensorRT = TensortRT{modelName, packagePath.string()};

		mModel = Model(modelName, {0}, {"bottle"}, mTensorRT.getInputTensorSize(), mTensorRT.getOutputTensorSize(), [](Model const& model, cv::Mat& rgbImage, cv::Mat& blobSizedImage, cv::Mat& blob) { preprocessYOLOv8Input(model, rgbImage, blobSizedImage, blob); }, [this](Model const& model, cv::Mat& output, std::vector<Detection>& detections) { parseYOLOv8Output(model, output, detections); });

		mUpperBound = cv::Vec3d(upperBoundH, upperBoundS, upperBoundV);
		mLowerBound = cv::Vec3d(lowerBoundH, lowerBoundS, lowerBoundV);

		// TODO: fix this next time! check wiki for what the message type should be
		imgSub = create_subscription<sensor_msgs::msg::PointCloud2>("/zed/left/points", 1, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) {
			LightDetector::imageCallback(msg);
		});
		
		imgPub = this->create_publisher<sensor_msgs::msg::Image>("/light_detector/img", 1);
		mDebugImgPub = this->create_publisher<sensor_msgs::msg::Image>("/light_detector/debug_img", 1);
		pointPub = this->create_publisher<geometry_msgs::msg::Vector3>("/light_detector/light_poses", 1);
	}

} // mrover