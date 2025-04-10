#include "light_detector.hpp"
#include <opencv2/core/matx.hpp>
#include <rcl/graph.h>
#include <rclcpp/node.hpp>

namespace mrover{

	LightDetector::LightDetector(rclcpp::NodeOptions const& options) : rclcpp::Node(NODE_NAME, options) {
		RCLCPP_INFO_STREAM(get_logger(),"Light Detector Initializing");

		std::string modelName;

		std::vector<ParameterWrapper> params{
                {"world_frame", mWorldFrame, "map"},
                {"light_detector/spiral_search_radius", SPIRAL_SEARCH_DIM, 50},
                {"light_detector/immediate_light_range", mImmediateLightRange, 5},
                {"light_detector/hit_increase", mHitIncrease, 5},
                {"light_detector/hit_decrease", mHitDecrease, 1},
                {"light_detector/hit_max", mHitMax, 100},
				{"light_detector/m_publish_threshold", mPublishThreshold, 50}
		};

        ParameterWrapper::declareParameters(this, params);
	}

	ColoredDetector::ColoredDetector(rclcpp::NodeOptions const& options) : LightDetector(options) {
		RCLCPP_INFO_STREAM(get_logger(), "Creating Colored Lights Detector...");

		std::vector<ParameterWrapper> params{
				{"camera_frame", mCameraFrame, "zed_left_camera_frame"},
				{"light_detector/lmodel_name", modelName, "25Epoch"},
                {"light_detector/model_score_threshold", mModelScoreThreshold, 0.3},
            	{"light_detector/odel_nms_threshold", mModelNMSThreshold, 0.5},
            	{"light_detector/debug", mDebug, true}
		};
		
        ParameterWrapper::declareParameters(this, params);

		//Get path for the model
		std::filesystem::path packagePath = std::filesystem::path{ament_index_cpp::get_package_prefix("mrover")} / ".." / ".." / "src" / "mrover";

		mTensorRT = TensortRT{modelName, packagePath.string()};
		
		using namespace std::placeholders;

        mModel = Model(modelName, {0, 0}, {"blue", "red"}, mTensorRT.getInputTensorSize(), mTensorRT.getOutputTensorSize(), [](Model const& model, cv::Mat& rgbImage, cv::Mat& blobSizedImage, cv::Mat& blob) { preprocessYOLOv8Input(model, rgbImage, blobSizedImage, blob); }, [this](Model const& model, cv::Mat& output, std::vector<Detection>& detections) { parseYOLOv8Output(model, output, detections); });

        RCLCPP_INFO_STREAM(get_logger(), std::format("Object detector initialized with model: {} and thresholds: {} and {}", mModel.modelName, mModelScoreThreshold, mModelNMSThreshold));
		
		imgSub = create_subscription<sensor_msgs::msg::PointCloud2>("/zed/left/points", 1, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) {
			ColoredDetector::pointCloudCallback(msg);
		});

		imgPub = this->create_publisher<sensor_msgs::msg::Image>("/light_detector/colored_img", 1);
	}

	InfraredDetector::InfraredDetector(rclcpp::NodeOptions const& options) : LightDetector(options) {
		RCLCPP_INFO_STREAM(get_logger(), "Creating Infrared Lights Detector...");
		
		std::vector<ParameterWrapper> params{
			// Infrared_frame is the same as the zed
			{"infrared_frame", mCameraFrame, "zed_left_camera_frame"},
		};
		
        ParameterWrapper::declareParameters(this, params);

		// CHANGE TOPIC NAME
		imgSub = create_subscription<sensor_msgs::msg::Image>("/infrared/points", 1, [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
			InfraredDetector::imageCallback(msg);
		});

		imgPub = this->create_publisher<sensor_msgs::msg::Image>("/light_detector/infrared_img", 1);
	}

} // mrover