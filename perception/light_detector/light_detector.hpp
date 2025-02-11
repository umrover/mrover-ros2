#pragma once
#include "lie.hpp"
#include <unordered_map>
#include "pch.hpp"

class PairHash{
	public:
		auto operator()(std::pair<int, int> const& p) const -> int{
			return p.first * 10000000 + p.second;
		}
};

namespace mrover {
	class LightDetector : public rclcpp::Node {
	protected:
    struct Detection {
        int classId{};
        std::string className;
        float confidence{};
        cv::Rect box;
        Detection(int _classId, std::string _className, float _confidence, cv::Rect _box) : classId{_classId}, className{_className}, confidence{_confidence}, box{_box} {}
    };

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

	private:
		cv::Mat mImgRGB;
		cv::Mat mImgHSV;
		cv::Mat mOutputImage;
        cv::Mat mRgbImage, mImageBlob;
		sensor_msgs::msg::Image mDetectionsImageMessage;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mDebugImgPub;

		int SPIRAL_SEARCH_DIM{};
		double mImmediateLightRange{};

		std::unordered_map<std::pair<double, double>, int, PairHash> mHitCounts;

		int mHitIncrease{};
		int mHitDecrease{};
		int mHitMax{};
		int mPublishThreshold{};
		float mModelScoreThreshold{};
        float mModelNMSThreshold{};
        bool mDebug{};

		// Model Detector
		TemprRT mTensorRT;

		// TF variables
		tf2_ros::Buffer mTfBuffer{get_clock()};
        tf2_ros::TransformBroadcaster mTfBroadcaster{this};
        tf2_ros::TransformListener mTfListener{mTfBuffer};

		std::string mCameraFrame;
        std::string mWorldFrame;

		// Thresholding Variables
		cv::Mat mThresholdedImg;
		cv::Vec3d mUpperBound;
		cv::Vec3d mLowerBound;

		cv::Mat mErodedImg;
		cv::Mat mDialtedImg;

		// Pub Sub
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr imgSub;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgPub;
		rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pointPub;

		// The number of lights that we push into the TF
        unsigned int numLightsSeen = 0;

		static constexpr char const* NODE_NAME = "light_detector";

		Model mModel;

		auto imageCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void;

		auto static convertPointCloudToRGB(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg, cv::Mat const& image) -> void;

		auto publishDetectedObjects(cv::InputArray image, std::vector<std::pair<int, int>> const& centroids) -> void;
		
		auto publishClosestLight(std::pair<double, double> &point) -> void;

		auto static rgb_to_hsv(cv::Vec3b const& rgb) -> cv::Vec3d;

		auto spiralSearchForValidPoint(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& cloudPtr, std::size_t u, std::size_t v, std::size_t width, std::size_t height) const -> std::optional<SE3d>;

		auto getPointFromPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& cloudPtr, std::pair<int, int> coordinates) -> std::optional<SE3d>;

		void increaseHitCount(std::optional<SE3d> const& light);

		void decreaseHitCounts();

		auto drawDetectionBoxes(cv::InputOutputArray image, std::span<Detection const> detections) -> void;

		auto publishDebugObjects(cv::InputArray const& image) -> void;

		auto caching() -> std::pair<std::pair<double, double>, bool>;

		auto calculateDistance(const std::pair<double, double> &p) -> double;

		auto getHitCount(std::optional<SE3d> const& light) -> int;

		void printHitCounts();

		auto round_to(double value, double precision) -> double;


        auto parseYOLOv8Output(Model const& model,
                               cv::Mat& output,
                               std::vector<Detection>& detections) const -> void;

		static auto preprocessYOLOv8Input(Model const& model, cv::Mat const& rgbImage, cv::Mat& blobSizedImage, cv::Mat& blob) -> void;

	public:
		auto onInit() -> void;

		explicit LightDetector(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

		~LightDetector() override = default;
	}; // LightDetector
} // mrover