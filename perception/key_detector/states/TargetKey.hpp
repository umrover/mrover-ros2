#pragma once
#include "../pch.hpp"
#include "../FSMContext.hpp"

namespace mrover {
    class TargetKey : public State {
    public:
        explicit TargetKey(std::shared_ptr<FSMContext> const& fsmContext);

        auto onLoop() -> State* override;

    private:
        rclcpp::Publisher<msg::IK>::SharedPtr mIkTargetPub;
        std::shared_ptr<FSMContext> mFSMContext;
        rclcpp::Rate sleepRate;
    };
} // namespace mrover
