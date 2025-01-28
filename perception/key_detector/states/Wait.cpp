#include "Wait.hpp"
#include <rclcpp/utilities.hpp>

namespace mrover {

    Wait::Wait(std::shared_ptr<FSMContext> const& fsmContext)
        : sleepRate(0.1), mFSMContext{fsmContext} {
    }

    auto Wait::onLoop() -> State* {
        if(mFSMContext->goal_handle->is_canceling()){
            return StateMachine::make_state<Cancel>(mFSMContext);
        }
        rclcpp::sleep_for(std::chrono::nanoseconds(1000));

        return StateMachine::make_state<TargetKey>(mFSMContext);
    }
} // namespace mrover
