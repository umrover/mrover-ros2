#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_tutorials_cpp/visibility_control.h"

class KeyActionServer : public rclcpp::Node
{
public:
  using KeyAction = action::KeyAction;
  using GoalHandleKeyAction = rclcpp_action::ServerGoalHandle<KeyAction>;

  explicit KeyActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
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

private:
  rclcpp_action::Server<KeyAction>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const KeyAction::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with code %d", goal->code);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleKeyAction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleKeyAction> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&KeyActionServer::execute, this, _1), goal_handle}.detach();
  }

};