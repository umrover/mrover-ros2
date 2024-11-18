#include "key_detector.hpp"

namespace mrover{
	KeyDetector::KeyDetector() : rclcpp::Node(NODE_NAME), mFSMTimer{create_wall_timer(std::chrono::milliseconds(1000), [&](){updateFSM();})}, mStateMachine{"Key Detector FSM", new State1()}, mStatePublisher{this, mStateMachine, "key_detector_fsm_structure", 10, "key_detector_fsm_state", 10}{
		RCLCPP_INFO_STREAM(get_logger(), "Creating KeyDetector Node...");

		mStateMachine.enableTransitions<State1, State1, State2>();
		mStateMachine.enableTransitions<State2, State1, State2>();
	}

	KeyDetector::~KeyDetector() = default;
}
