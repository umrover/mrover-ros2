#include "light_detector.hpp"

namespace mrover {
	auto LightDetector::pointCloudCallback(sensor_msgs::msg::PointCloud2 const& msg) -> void{
		// Put any image processing code here!
		
		RCLCPP_INFO_STREAM(get_logger(), "Image Callback...");
	}
}

