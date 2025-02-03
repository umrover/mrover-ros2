#include "Off.hpp"

namespace mrover {
    Off::Off(std::shared_ptr<FSMContext> const& fsmContext) : mFSMContext(fsmContext) {}

    auto Off::onLoop() -> State* {
        RCLCPP_INFO(mFSMContext->node->get_logger(), "Off Onloop");
        //set the velocity of arm to zero
        if(mFSMContext->restart){
            mFSMContext->restart = false;
            return StateMachine::make_state<TargetKey>(mFSMContext);
        }

        return this;
    }
} // namespace mrover
