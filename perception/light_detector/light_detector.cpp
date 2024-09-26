#include "light_detector.hpp"

namespace mrover {
	LightDetector::LightDetector() : rclcpp::Node(NODE_NAME){
		// Put any node initialization code here!
		
		mPointCloudSub = create_subscription<sensor_msgs::msg::PointCloud2>("camera/left/points", 1, [this](sensor_msgs::msg::PointCloud2 const& msg){
				LightDetector::pointCloudCallback(msg);
				});
		
		RCLCPP_INFO_STREAM(get_logger(), "Initializing the Light Detector Node...");
	}

	LightDetector::~LightDetector() = default;
}



auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::LightDetector>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
