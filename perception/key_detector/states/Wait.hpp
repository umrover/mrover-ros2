#pragma once
#include "../pch.hpp"
#include "../FSMContext.hpp"

namespace mrover {
    class Wait : public State {
    public:
        using KeyAction = mrover::action::KeyAction;
        using GoalHandleKeyAction = rclcpp_action::ServerGoalHandle<KeyAction>;

        explicit Wait(std::shared_ptr<FSMContext> const& mFSMContext);

        auto onLoop() -> State* override;

    private:
        rclcpp::Rate sleepRate;
        std::shared_ptr<FSMContext> const mFSMContext;
    };
} // namespace mrover
