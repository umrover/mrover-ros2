
#include "key_detector.hpp"

  using KeyActionServer = mrover::KeyActionServer;

  KeyActionServer::KeyActionServer(const rclcpp::NodeOptions & options)
  : Node("KeyAction_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<KeyAction>(
      this,
      "KeyAction",
      std::bind(&KeyActionServer::handle_goal, this, _1, _2),
      std::bind(&KeyActionServer::handle_cancel, this, _1),
      std::bind(&KeyActionServer::handle_accepted, this, _1));
  }

  rclcpp_action::GoalResponse mrover::KeyActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const KeyAction::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with code %s", goal->code.data());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse KeyActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleKeyAction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void KeyActionServer::execute(const std::shared_ptr<GoalHandleKeyAction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<KeyAction::Feedback>();
    auto result = std::make_shared<KeyAction::Result>();

    //Todo: whatever is needed to move (Use IK)
    

    // Check if movement is done
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  void KeyActionServer::handle_accepted(const std::shared_ptr<GoalHandleKeyAction> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&KeyActionServer::execute, this, _1), goal_handle}.detach();
  }