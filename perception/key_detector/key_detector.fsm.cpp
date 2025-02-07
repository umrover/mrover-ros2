#include "key_detector.hpp"
#include <rclcpp/logging.hpp>

namespace mrover{
	void KeyDetector::updateFSM(){
		RCLCPP_INFO_STREAM(get_logger(), "FSM Callback...");
		RCLCPP_INFO_STREAM(get_logger(), mStateMachine.getCurrentState());
		mStateMachine.update();
	}
}
