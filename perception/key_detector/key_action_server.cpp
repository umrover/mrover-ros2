
#include "key_detector.hpp"

using KeyDetector = mrover::KeyDetector;  


rclcpp_action::GoalResponse mrover::KeyDetector::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const KeyAction::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with code %s", goal->code.data());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse KeyDetector::handle_cancel(
  const std::shared_ptr<GoalHandleKeyAction> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void KeyDetector::execute(const std::shared_ptr<GoalHandleKeyAction> goal_handle)
{
  
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  auto result = std::make_shared<KeyAction::Result>();

  fsm_ctx->goal_handle = goal_handle;
  fsm_ctx->curr_key_index = 0;
  fsm_ctx->node = this->shared_from_this();

  // start state
  while (rclcpp::ok()){
    rclcpp::Rate loop_rate(1);
    auto feedback = std::make_shared<KeyAction::Feedback>();


    updateFSM();

    loop_rate.sleep();
  }



  // Check if movement is done
  if (rclcpp::ok()) {
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }



}

void KeyDetector::handle_accepted(const std::shared_ptr<GoalHandleKeyAction> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&KeyDetector::execute, this, _1), goal_handle}.detach();
}