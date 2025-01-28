#pragma once
#include "pch.hpp"

namespace mrover {
    class KeyDetector : public rclcpp::Node {
    public:
        explicit KeyDetector(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        ~KeyDetector() override;

    private:
        static constexpr char const* NODE_NAME = "key_detector";

        rclcpp::TimerBase::SharedPtr mFSMTimer;

        std::shared_ptr<FSMContext> mFSMContext;

        bool mIsStateMachineEnabled;
        StateMachine mStateMachine;

        StatePublisher mStatePublisher;

        rclcpp_action::Server<KeyAction>::SharedPtr action_server_;

        rclcpp_action::GoalResponse handle_goal(
                rclcpp_action::GoalUUID const& uuid,
                std::shared_ptr<KeyAction::Goal const> goal);

        rclcpp_action::CancelResponse handle_cancel(
                std::shared_ptr<GoalHandleKeyAction> const goal_handle);

        void execute();

        void handle_accepted(std::shared_ptr<GoalHandleKeyAction> const goal_handle);
    };


} // namespace mrover
