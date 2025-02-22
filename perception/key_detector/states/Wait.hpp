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
        constexpr static double WAIT_DURATION = 1;
        static constexpr double KEYBOARD_TYPING_HEIGHT = 0.2;
        static constexpr double KEYBOARD_PRESS_DISTANCE = 0.05;
        static constexpr double FINGER_OFFSET = 0.05 - 0.01725;
        static constexpr double SPEED = 3;
        rclcpp::Rate sleepRate;
        std::shared_ptr<FSMContext> const mFSMContext;
    };
} // namespace mrover
