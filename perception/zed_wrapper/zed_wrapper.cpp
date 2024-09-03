#include "zed_wrapper.hpp"

namespace mrover {
	ZedWrapper::ZedWrapper() : Node(NODE_NAME){
		RCLCPP_INFO(this->get_logger(), "Created Zed Wrapper Node, %s", NODE_NAME);

		// Declare Params
        auto paramSub = std::make_shared<rclcpp::ParameterEventHandler>(this);

		int imageWidth{};
		int imageHeight{};

		std::string svoFile{};

		std::vector<ParameterWrapper> params{
			{"depth_confidence", mDepthConfidence},
			{"serial_number", mSerialNumber},
			{"grab_target_fps", mGrabTargetFps},
			{"texture_confidence", mTextureConfidence},
			{"image_width", imageWidth},
			{"image_height", imageHeight},
			{"svo_file", svoFile},
			{}
		};

		ParameterWrapper::declareParameters(this, paramSub, params);


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

		initParameters.depth_stabilization = mUseDepthStabilization;
		initParameters.camera_resolution = stringToZedEnum<sl::RESOLUTION>(grabResolutionString);
		initParameters.depth_mode = stringToZedEnum<sl::DEPTH_MODE>(depthModeString);
		initParameters.coordinate_units = sl::UNIT::METER;
		initParameters.sdk_verbose = true; // Log useful information
		initParameters.camera_fps = mGrabTargetFps;
		initParameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD; // Match ROS
		initParameters.depth_maximum_distance = mDepthMaximumDistance;

		mDepthEnabled = initParameters.depth_mode != sl::DEPTH_MODE::NONE;

		if (mZed.open(initParameters) != sl::ERROR_CODE::SUCCESS) {
			throw std::runtime_error("ZED failed to open");
		}
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
