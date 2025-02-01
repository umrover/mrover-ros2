#include "Cancel.hpp"

namespace mrover {
    Cancel::Cancel(std::shared_ptr<FSMContext> const& fsmContext) : mFSMContext(fsmContext) {}

    auto Cancel::onLoop() -> State* {
        RCLCPP_INFO(mFSMContext->node->get_logger(), "Cancel Onloop");

        auto result = std::make_shared<KeyAction::Result>();
        result->success = false;
        mFSMContext->goal_handle->canceled(result);
        RCLCPP_INFO(mFSMContext->node->get_logger(), "Goal failed");

        return StateMachine::make_state<Off>(mFSMContext);
    }
} // namespace mrover
