#pragma once

#include "pch.hpp"


namespace mrover {


// class KeyDetector : public rclcpp::Node
// {
//   public:  
//     explicit KeyDetector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
//   private:
//     rclcpp_action::Server<KeyAction>::SharedPtr action_server_;

//     rclcpp_action::GoalResponse handle_goal(
//       const rclcpp_action::GoalUUID & uuid,
//       std::shared_ptr<const KeyAction::Goal> goal)
//   ;

//     rclcpp_action::CancelResponse handle_cancel(
//       const std::shared_ptr<GoalHandleKeyAction> goal_handle);

//     void execute(const std::shared_ptr<GoalHandleKeyAction> goal_handle);

//     void handle_accepted(const std::shared_ptr<GoalHandleKeyAction> goal_handle);
// };

class KeyDetector : public rclcpp::Node{
  public:

    explicit KeyDetector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~KeyDetector() override;

  private:
    static constexpr char const* NODE_NAME = "key_detector";

    rclcpp::TimerBase::SharedPtr mFSMTimer;

    std::shared_ptr<FSMCtx> fsm_ctx;

    StateMachine mStateMachine;

    StatePublisher mStatePublisher;


    void updateFSM();


    rclcpp_action::Server<KeyAction>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const KeyAction::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleKeyAction> goal_handle);

    void execute(const std::shared_ptr<GoalHandleKeyAction> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleKeyAction> goal_handle);

};


} // namespace mrover
