#pragma once
#include "../pch.hpp"
#include "../FSMContext.hpp"

namespace mrover {
    class TargetKey : public State {
    public:
        explicit TargetKey(std::shared_ptr<FSMContext> const& fsmContext);

        auto onLoop() -> State* override;

    private:
        static constexpr double KEY_TOLERANCE = 0.01;
        static constexpr int MAX_KEY_HITCOUNT = 5;
        static constexpr double KEYBOARD_TYPING_HEIGHT = 0.2;
        static constexpr double FINGER_OFFSET = 0.05 - 0.01725;
        static constexpr double SPEED = 3;
        rclcpp::Publisher<msg::IK>::SharedPtr mIkTargetPub;
        std::shared_ptr<FSMContext> mFSMContext;
        rclcpp::Rate sleepRate;
        int mToleranceHitcount;
    };
} // namespace mrover
