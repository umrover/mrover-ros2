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

	private:
		cv::Mat mImgRGB;
		cv::Mat mImgHSV;
		cv::Mat mOutputImage;

		int SPIRAL_SEARCH_DIM{};

		double mImmediateLightRange{};

		std::unordered_map<std::pair<double, double>, int, PairHash> mHitCounts;

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

		auto imageCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void;

		auto static convertPointCloudToRGB(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg, cv::Mat const& image) -> void;

		auto publishDetectedObjects(cv::InputArray image, std::vector<std::pair<int, int>> const& centroids) -> void;
		
		auto publishClosestLight(std::pair<double, double> &point) -> void;

		auto static rgb_to_hsv(cv::Vec3b const& rgb) -> cv::Vec3d;

		auto spiralSearchForValidPoint(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& cloudPtr, std::size_t u, std::size_t v, std::size_t width, std::size_t height) const -> std::optional<SE3d>;

		auto getPointFromPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& cloudPtr, std::pair<int, int> coordinates) -> std::optional<SE3d>;

		void increaseHitCount(std::optional<SE3d> const& light);

		void decreaseHitCounts();

		auto caching() -> std::pair<std::pair<double, double>, bool>;

		auto calculateDistance(const std::pair<double, double> &p) -> double;

		auto getHitCount(std::optional<SE3d> const& light) -> int;

		void printHitCounts();

		auto round_to(double value, double precision) -> double;

	public:
		auto onInit() -> void;

		explicit LightDetector(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

		~LightDetector() override = default;
	}; // LightDetector
} // mrover