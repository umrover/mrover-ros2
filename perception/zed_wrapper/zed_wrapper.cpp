#include "zed_wrapper.hpp"

namespace mrover {
	ZedWrapper::ZedWrapper() : Node(NODE_NAME){
		RCLCPP_INFO(this->get_logger(), "Created Zed Wrapper Node, %s", NODE_NAME);
	}


	auto ZedWrapper::grabThread() -> void{
		RCLCPP_INFO(this->get_logger(), "Starting grab thread");
		while(rclcpp::ok()){
			sl::RuntimeParameters runtimeParameters;
		}
	}

	auto pointCloudUpdateThread() -> void{

	}
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mrover::ZedWrapper>());
  rclcpp::shutdown();
  return 0;
}
