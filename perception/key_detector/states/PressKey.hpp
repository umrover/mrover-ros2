#pragma once
#include "../pch.hpp"
#include "../FSMContext.hpp"

namespace mrover {
    class PressKey : public State {
    public:
        using KeyAction = mrover::action::KeyAction;
        using GoalHandleKeyAction = rclcpp_action::ServerGoalHandle<KeyAction>;

        explicit PressKey(std::shared_ptr<FSMContext> const& fsmContext);

        auto onLoop() -> State* override;

    private:
        std::shared_ptr<FSMContext> const mFSMContext;
    };
} // namespace mrover
