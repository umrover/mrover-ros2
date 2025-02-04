#include "key_detector.hpp"
#include "states/TargetKey.hpp"
namespace mrover {
    KeyDetector::KeyDetector(rclcpp::NodeOptions const& options) : rclcpp::Node(NODE_NAME, options), mFSMTimer{create_wall_timer(std::chrono::milliseconds(1000), [&](){execute();})}, mFSMContext(std::make_shared<FSMContext>(mTfBuffer)), mStateMachine{"Key Detector FSM", StateMachine::make_state<Off>(mFSMContext)}, mStatePublisher{this, mStateMachine, "key_detector_fsm_structure", 10, "key_detector_fsm_state", 10} {
        RCLCPP_INFO_STREAM(get_logger(), "Creating KeyDetector Node...");

        mArmVelocityPub = create_publisher<geometry_msgs::msg::Vector3>("/ee_vel_cmd", 1);
        mFSMContext->armVelocityPub = mArmVelocityPub;

        mStateMachine.enableTransitions<TargetKey, TargetKey, PressKey, Cancel, Done>();
        mStateMachine.enableTransitions<PressKey, PressKey, Wait, Cancel>();
        mStateMachine.enableTransitions<Wait, TargetKey, Wait, Cancel>();
        mStateMachine.enableTransitions<Cancel, Off, Cancel>();
        mStateMachine.enableTransitions<Off, Off, TargetKey, Cancel>();
        mStateMachine.enableTransitions<Done, Off>();

        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<KeyAction>(
                this,
                "KeyAction",
                std::bind(&KeyDetector::handle_goal, this, _1, _2),
                std::bind(&KeyDetector::handle_cancel, this, _1),
                std::bind(&KeyDetector::handle_accepted, this, _1));
    }

    KeyDetector::~KeyDetector() = default;
} // namespace mrover
