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

    std::shared_ptr<tf2_ros::Buffer> mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    std::shared_ptr<tf2_ros::TransformListener> mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);

    void updateFSM();

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mIkTargetPub;
    
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
