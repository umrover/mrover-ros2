#include "zed_wrapper.hpp"

namespace mrover {
	ZedWrapper::ZedWrapper() : Node(NODE_NAME){
		RCLCPP_INFO(this->get_logger(), "Created Zed Wrapper Node, %s", NODE_NAME);

		// Declare Params
        auto paramSub = std::make_shared<rclcpp::ParameterEventHandler>(this);

		//TODO: Refactor this to something more portable

		int imageWidth{};
		int imageHeight{};

		std::vector<ParameterWrapper> params{
			{"image_width", imageWidth},
			{"image_height", imageHeight}
		};

		ParameterWrapper::declareParameters(this, paramSub, params);


		/*
		std::map<std::string, int> integerParams{
			{"depth_confidence", 70},
			{"serial_number", -1},
			{"grab_target_fps", 60},
			{"texture_confidence", 100},
			{"image_width", 1280},
			{"image_height", 720}
		};

		this->declare_parameters("", integerParams);

		std::map<std::string, int&> integerVariables{
			{"depth_confidence", mDepthConfidence},
			{"serial_number", mSerialNumber},
			{"grab_target_fps", mGrabTargetFps},
			{"texture_confidence", mTextureConfidence},
			{"image_width", imageWidth},
			{"image_height", imageHeight}
		};

		for(auto& [descriptor, variable] : integerVariables){
			get_parameter(descriptor, variable);
			paramSub->add_parameter_callback(descriptor, [&](rclcpp::Parameter const& param) {
				variable = static_cast<int>(param.as_int());
			});
		}

		std::string svoFile{};

		std::map<std::string, std::string> stringParams{
			{"svo_file", {}}
		};

		this->declare_parameters("", stringParams);


		std::map<std::string, std::string&> stringVariables{
			{"svo_file", svoFile}
		};

		for(auto& [descriptor, variable] : stringVariables){
			get_parameter(descriptor, variable);
			paramSub->add_parameter_callback(descriptor, [&](rclcpp::Parameter const& param) {
				RCLCPP_INFO(this->get_logger(), "Recieved %s", param.as_string().c_str());
				variable = param.as_string();
			});
		}

		if (imageWidth < 0 || imageHeight < 0) {
			throw std::invalid_argument("Invalid image dimensions");
		}
		if (mGrabTargetFps < 0) {
			throw std::invalid_argument("Invalid grab target framerate");
		}

		sl::InitParameters initParameters;
		if (svoFile.c_str()) {
			initParameters.input.setFromSVOFile(svoFile.c_str());
		} else {
			if (mSerialNumber == -1) {
				initParameters.input.setFromCameraID(-1, sl::BUS_TYPE::USB);
			} else {
				initParameters.input.setFromSerialNumber(mSerialNumber, sl::BUS_TYPE::USB);
			}
		}

		//initParameters.depth_stabilization = mUseDepthStabilization;
		//initParameters.camera_resolution = stringToZedEnum<sl::RESOLUTION>(grabResolutionString);
		//initParameters.depth_mode = stringToZedEnum<sl::DEPTH_MODE>(depthModeString);
		initParameters.coordinate_units = sl::UNIT::METER;
		initParameters.sdk_verbose = true; // Log useful information
		initParameters.camera_fps = mGrabTargetFps;
		initParameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD; // Match ROS
		//initParameters.depth_maximum_distance = mDepthMaximumDistance;

		//mDepthEnabled = initParameters.depth_mode != sl::DEPTH_MODE::NONE;

		if (mZed.open(initParameters) != sl::ERROR_CODE::SUCCESS) {
			throw std::runtime_error("ZED failed to open");
		}
		*/
	}


	auto ZedWrapper::grabThread() -> void{
		RCLCPP_INFO(this->get_logger(), "Starting grab thread");
		while(rclcpp::ok()){
			sl::RuntimeParameters runtimeParameters;
			runtimeParameters.confidence_threshold = mDepthConfidence;
			runtimeParameters.texture_confidence_threshold = mTextureConfidence;

			if (sl::ERROR_CODE error = mZed.grab(runtimeParameters); error != sl::ERROR_CODE::SUCCESS)
                    throw std::runtime_error(std::format("ZED failed to grab {}", sl::toString(error).c_str()));
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
