#include "zed_wrapper.hpp"

namespace mrover {
	ZedWrapper::ZedWrapper() : Node(NODE_NAME){
		RCLCPP_INFO(this->get_logger(), "Created Zed Wrapper Node, %s", NODE_NAME);

		// Declare Params
        auto paramSub = std::make_shared<rclcpp::ParameterEventHandler>(this);

		std::map<std::string, int> integerParams{
			{"depth_confidence", 70},
			{"serial_number", -1},
			{"grab_target_fps", 60},
			{"texture_confidence", 100}
		};

		this->declare_parameters("", integerParams);

		std::map<std::string, int&> integerVariables{
			{"depth_confidence", mDepthConfidence},
			{"serial_number", mSerialNumber},
			{"grab_target_fps", mGrabTargetFps},
			{"texture_confidence", mTextureConfidence}
		};

		for(auto& [descriptor, variable] : integerVariables){
			get_parameter(descriptor, variable);
			paramSub->add_parameter_callback(descriptor, [&](rclcpp::Parameter const& param) {
				RCLCPP_INFO(this->get_logger(), "Recieved %d", static_cast<int>(param.as_int()));
				variable = static_cast<int>(param.as_int());
			});
		}

		grabThread();

	}


	auto ZedWrapper::grabThread() -> void{
		RCLCPP_INFO(this->get_logger(), "Starting grab thread");
		while(rclcpp::ok()){
			sl::RuntimeParameters runtimeParameters;
			runtimeParameters.confidence_threshold = mDepthConfidence;
			RCLCPP_INFO(this->get_logger(), "%d\n", mDepthConfidence);
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
