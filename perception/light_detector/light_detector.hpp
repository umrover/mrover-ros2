#include "pch.hpp"

namespace mrover {
	class LightDetector : public rclcpp::Node {
	private:
		static constexpr char const* NODE_NAME = "light_detector";

		std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> mPointCloudSub;

		auto pointCloudCallback(sensor_msgs::msg::PointCloud2 const& msg) -> void;

	public:
		explicit LightDetector();

		~LightDetector() override;
	};
}
