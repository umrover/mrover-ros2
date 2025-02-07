#pragma once
#include "lie.hpp"
#include "pch.hpp"
#include <unordered_map>

class PairHash{
	public:
		auto operator()(std::pair<int, int> const& p) const -> int{
			return p.first * 10000000 + p.second;
		}
};

namespace mrover {
	class LightDetector : public rclcpp::Node {

	protected:
		cv::Mat mImgRGB, mImageBlob;;
		cv::Mat mOutputImage;

		std::unordered_map<std::pair<double, double>, int, PairHash> mHitCounts;

		// Params
		int SPIRAL_SEARCH_DIM{};
		double mImmediateLightRange{};
		int mHitIncrease{};
		int mHitDecrease{};
		int mHitMax{};
		int mPublishThreshold{};

		// TF variables
		tf2_ros::Buffer mTfBuffer{get_clock()};
        tf2_ros::TransformBroadcaster mTfBroadcaster{this};
        tf2_ros::TransformListener mTfListener{mTfBuffer};

		std::string mCameraFrame;
        std::string mWorldFrame;

		// Pub Sub
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgPub;
		rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pointPub;

		// The number of lights that we push into the TF
        unsigned int numLightsSeen = 0;

		// Node Name
		static constexpr char const* NODE_NAME = "light_detector";
		

		auto spiralSearchForValidPoint(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& cloudPtr, std::size_t u, std::size_t v, std::size_t width, std::size_t height) const -> std::optional<SE3d>;

		auto caching() -> std::pair<std::pair<double, double>, bool>;
		
		
		// KEEP THE BELOW
		auto static round_to(double value, double precision) -> double;

		auto static calculateDistance(const std::pair<double, double> &p) -> double;

		void increaseHitCount(std::optional<SE3d> const& light);

		void decreaseHitCounts();

		auto getHitCount(std::optional<SE3d> const& light) -> int;

		auto publishClosestLight(std::pair<double, double> &point) -> void;

	public:
		explicit LightDetector(rclcpp::NodeOptions const& options);
		~LightDetector() override = default;
	}; // LightDetector


	class InfraredDetector final : public LightDetector {
		// similar but subscribe to different sensor and have different processing algorithm
		// similar to object detector
		// shared stuff with colored detector
		//just change and specify sensor subscriber and how to perform the callback
		private:
			// Subscribers (Infrared Camera)
			rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgSub;
		public:
			explicit InfraredDetector(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

			auto imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;

			auto publishDetectedObjects(cv::InputArray image, std::vector<std::pair<int, int>> const& centroids) -> void;
		
	};
	
	
	class ColoredDetector final : public LightDetector {
		// accepts ros parameters to define what we want to run
		private:
			float mModelScoreThreshold{};
        	float mModelNMSThreshold{};
        	bool mDebug{};
			std::string modelName{};

			// Model Detector
			TensortRT mTensorRT;

			// Thresholding Variables
			cv::Mat mThresholdedImg;
			cv::Vec3d mUpperBound;
			cv::Vec3d mLowerBound;

			// Temp Variables for Formatting Img
			cv::Mat mErodedImg;
			cv::Mat mDialtedImg;

			// Subscribers (ZED Camera)
			rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr imgSub;

			// struct Detection {
			// 	int classId{};
			// 	std::string className;
			// 	float confidence{};
			// 	cv::Rect box;
	
			// 	Detection(int _classId, std::string _className, float _confidence, cv::Rect _box) : classId{_classId}, className{_className}, confidence{_confidence}, box{_box} {}
			// };
	
			struct Model {
				std::string modelName;
	
				std::vector<int> objectHitCounts;
	
				std::vector<std::string> classes;
	
				std::vector<int64_t> inputTensorSize;
	
				std::vector<int64_t> outputTensorSize;
	
				// Converts an rgb image to a blob
				std::function<void(Model const&, cv::Mat&, cv::Mat&, cv::Mat&)> rgbImageToBlob;
	
				// Converts an output tensor into a vector of detections
				std::function<void(Model const&, cv::Mat&, std::vector<Detection>&)> outputTensorToDetections;
	
				Model() = default;
	
				Model(std::string _modelName, std::vector<int> _objectHitCounts, std::vector<std::string> _classes, std::vector<int64_t> _inputTensorSize, std::vector<int64_t> _outputTensorSize, std::function<void(Model const&, cv::Mat&, cv::Mat&, cv::Mat&)> _rbgImageToBlob, std::function<void(Model const&, cv::Mat&, std::vector<Detection>&)> _outputTensorToDetections) : modelName{std::move(_modelName)}, objectHitCounts(std::move(_objectHitCounts)), classes(std::move(_classes)), inputTensorSize(std::move(_inputTensorSize)), outputTensorSize(std::move(_outputTensorSize)), rgbImageToBlob{std::move(_rbgImageToBlob)}, outputTensorToDetections{std::move(_outputTensorToDetections)} {}
			};

			Model mModel;
			
		public:

			explicit ColoredDetector(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

			auto pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void;

			auto publishDetectedObjects(cv::InputArray image, std::vector<std::pair<int, int>> const& centroids) -> void;
		
			auto getPointFromPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& cloudPtr, std::pair<int, int> coordinates) -> std::optional<SE3d>;

			auto static convertPointCloudToRGB(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg, cv::Mat const& image) -> void;

			auto static drawDetectionBoxes(cv::InputOutputArray image, std::span<Detection const> detections) -> void;

			auto parseYOLOv8Output(Model const& model,
				cv::Mat& output,
				std::vector<Detection>& detections) const -> void;

			static auto preprocessYOLOv8Input(Model const& model, cv::Mat const& rgbImage, cv::Mat& blobSizedImage, cv::Mat& blob) -> void;
	
			auto updateHitsObject(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg,
				std::span<Detection const> detections,
				cv::Size const& imageSize = {640, 640}) -> void;
		};
} // mrover