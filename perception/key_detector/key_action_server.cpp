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
    if(!mFSMContext->node){
        mFSMContext->node = this->shared_from_this(); // THIS IS COOKED BC std::bad_weak_ptr is called in init list
    }

    if (rclcpp::ok()) {
        rclcpp::Rate loop_rate(1);
        auto feedback = std::make_shared<KeyAction::Feedback>();

        // Perform loop for the state machine
        mStateMachine.update();

        loop_rate.sleep();
    }
}

void KeyDetector::handle_accepted(std::shared_ptr<GoalHandleKeyAction> const goal_handle) {
    mFSMContext->restart = true;
    mFSMContext->goal_handle = goal_handle;
    mFSMContext->curr_key_index = 0;
}
