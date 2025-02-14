#include "key_detector.hpp"
#include "states/TargetKey.hpp"
namespace mrover{
	KeyDetector::KeyDetector(rclcpp::NodeOptions const& options) : rclcpp::Node(NODE_NAME, options), mFSMTimer{create_wall_timer(std::chrono::milliseconds(1000), [&](){updateFSM();})}, fsm_ctx(std::make_shared<FSMCtx>()), mStateMachine{"Key Detector FSM", StateMachine::make_state<Off>(fsm_ctx)}, mStatePublisher{this, mStateMachine, "key_detector_fsm_structure", 10, "key_detector_fsm_state", 10}{
		RCLCPP_INFO_STREAM(get_logger(), "Creating KeyDetector Node...");
        
		mStateMachine.enableTransitions<TargetKey, TargetKey, PressKey, Cancel, Off>();
		mStateMachine.enableTransitions<PressKey, PressKey, Wait, Cancel>();
		mStateMachine.enableTransitions<Wait, TargetKey, Wait, Cancel>();
		mStateMachine.enableTransitions<Cancel, Off>();
		mStateMachine.enableTransitions<Off, Off, TargetKey>();
		using namespace std::placeholders;

		mIkTargetPub = create_publisher<geometry_msgs::msg::Vector3>("ee_vel_cmd", 1);

		fsm_ctx->mTfBuffer = mTfBuffer;
		fsm_ctx->mIkTargetPub = mIkTargetPub;
		fsm_ctx->init = false;
		fsm_ctx->fail = false;

		this->action_server_ = rclcpp_action::create_server<KeyAction>(
			this,
			"KeyAction",
			std::bind(&KeyDetector::handle_goal, this, _1, _2),
			std::bind(&KeyDetector::handle_cancel, this, _1),
			std::bind(&KeyDetector::handle_accepted, this, _1));
	}

	KeyDetector::~KeyDetector() = default;
}