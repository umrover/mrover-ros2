#include "Done.hpp"

namespace mrover {
    Done::Done(std::shared_ptr<FSMContext> const& fsmContext) : mFSMContext(fsmContext) {}

    auto Done::onLoop() -> State* {
        RCLCPP_INFO(mFSMContext->node->get_logger(), "Done Onloop");

        auto result = std::make_shared<KeyAction::Result>();
        result->success = true;
        mFSMContext->goal_handle->succeed(result);
        RCLCPP_INFO(mFSMContext->node->get_logger(), "Goal succeeded");

        return StateMachine::make_state<Off>(mFSMContext);
    }
} // namespace mrover
