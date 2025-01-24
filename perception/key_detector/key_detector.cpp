#include "key_detector.hpp"
namespace mrover{
	KeyDetector::KeyDetector(rclcpp::NodeOptions const& options) : rclcpp::Node(NODE_NAME), mFSMTimer{create_wall_timer(std::chrono::milliseconds(1000), [&](){updateFSM();})}, fsm_ctx(std::make_shared<FSMCtx>()), mStateMachine{"Key Detector FSM", new TargetKey(fsm_ctx)}, mStatePublisher{this, mStateMachine, "key_detector_fsm_structure", 10, "key_detector_fsm_state", 10}{
		RCLCPP_INFO_STREAM(get_logger(), "Creating KeyDetector Node...");

		// mStateMachine.enableTransitions<State1, State1, State2>();
		// mStateMachine.enableTransitions<State2, State1, State2>();
		using namespace std::placeholders;

		this->action_server_ = rclcpp_action::create_server<KeyAction>(
			this,
			"KeyAction",
			std::bind(&KeyDetector::handle_goal, this, _1, _2),
			std::bind(&KeyDetector::handle_cancel, this, _1),
			std::bind(&KeyDetector::handle_accepted, this, _1));

	}

	KeyDetector::~KeyDetector() = default;
}