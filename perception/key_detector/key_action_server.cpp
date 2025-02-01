#include "key_detector.hpp"

using KeyDetector = mrover::KeyDetector;

rclcpp_action::GoalResponse mrover::KeyDetector::handle_goal(
        rclcpp_action::GoalUUID const& uuid,
        std::shared_ptr<KeyAction::Goal const> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with code %s", goal->code.data());
    (void) uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse KeyDetector::handle_cancel(
        std::shared_ptr<GoalHandleKeyAction> const goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void) goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void KeyDetector::execute() {
    RCLCPP_INFO_STREAM(get_logger(), "Running State Machine Execute...\n");
    if (rclcpp::ok() && (mStateMachine.getCurrentState() != "Cancel" && mStateMachine.getCurrentState() != "Done")) {
    RCLCPP_INFO_STREAM(get_logger(), "Running State Machine Execute...\n");
        rclcpp::Rate loop_rate(1);
    RCLCPP_INFO_STREAM(get_logger(), "Running State Machine Execute...\n");
        auto feedback = std::make_shared<KeyAction::Feedback>();
    RCLCPP_INFO_STREAM(get_logger(), "Running State Machine Execute...\n");

        // Perform loop for the state machine
        mStateMachine.update();

        loop_rate.sleep();
    }

    // Check if movement is done
    if (rclcpp::ok() && mStateMachine.getCurrentState() == "Done") {
        auto result = std::make_shared<KeyAction::Result>();
        result->success = true;
        mFSMContext->goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }else if (rclcpp::ok() && mStateMachine.getCurrentState() == "Cancel"){
        auto result = std::make_shared<KeyAction::Result>();
        result->success = false;
        mFSMContext->goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal failed");
    }
}

void KeyDetector::handle_accepted(std::shared_ptr<GoalHandleKeyAction> const goal_handle) {
    mFSMContext->restart = true;
    mFSMContext->goal_handle = goal_handle;
    mFSMContext->node = this->shared_from_this();
    mFSMContext->curr_key_index = 0;

    mStateMachine.setState(StateMachine::make_state<TargetKey>(mFSMContext));
}
