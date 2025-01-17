#pragma once

#include "pch.hpp"


namespace mrover {


class KeyActionServer : public rclcpp::Node
{
public:
  using KeyAction = mrover::action::KeyAction;
  using GoalHandleKeyAction = rclcpp_action::ServerGoalHandle<KeyAction>;
  

  explicit KeyActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
private:
  rclcpp_action::Server<KeyAction>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const KeyAction::Goal> goal)
;

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleKeyAction> goal_handle);

  void execute(const std::shared_ptr<GoalHandleKeyAction> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleKeyAction> goal_handle);

};
	class KeyDetector : public rclcpp::Node{
	public:
    using KeyAction = mrover::action::KeyAction;
    using GoalHandleKeyAction = rclcpp_action::ServerGoalHandle<KeyAction>;

		KeyDetector(const std::shared_ptr<GoalHandleKeyAction> _goal);
    

		~KeyDetector() override;

	private:
    static constexpr char const* NODE_NAME = "key_detector";

		rclcpp::TimerBase::SharedPtr mFSMTimer;

		StateMachine mStateMachine;

		StatePublisher mStatePublisher;

    const std::shared_ptr<GoalHandleKeyAction> goal;

		void updateFSM();
	};

	

} // namespace mrover
