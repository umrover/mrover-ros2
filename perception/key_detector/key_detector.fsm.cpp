#include "key_detector.hpp"

namespace mrover{
	void KeyDetector::updateFSM(){
		RCLCPP_INFO_STREAM(get_logger(), "FSM Callback...");

		mStateMachine.update();
	}
}
