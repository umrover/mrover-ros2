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
        enum PRESS_KEY_STATE {
            EXTEND = 0,
            RETRACT = 1
        };

        static constexpr double KEY_TOLERANCE = 0.01;
        static constexpr int MAX_KEY_HITCOUNT = 5;
        static constexpr double KEYBOARD_TYPING_HEIGHT = 0.2;
        static constexpr double KEYBOARD_PRESS_DISTANCE = 0.05;
        static constexpr double FINGER_OFFSET = 0.05 - 0.01725;
        static constexpr double SPEED = 3;
        std::shared_ptr<FSMContext> const mFSMContext;
        int mToleranceHitcount;
        PRESS_KEY_STATE mState;
    };
} // namespace mrover
