#include "PressKey.hpp"

namespace mrover {
    PressKey::PressKey(std::shared_ptr<FSMContext> const& fsmContext) : mFSMContext(fsmContext) {}

    auto PressKey::onLoop() -> State* {
        if(mFSMContext->goal_handle->is_canceling()){
            return StateMachine::make_state<Cancel>(mFSMContext);
        }

        ++mFSMContext->curr_key_index;

        if(static_cast<std::size_t>(mFSMContext->curr_key_index) == mFSMContext->goal_handle->get_goal()->code.size()){
            return StateMachine::make_state<Done>(mFSMContext);
        }else{
            return StateMachine::make_state<Wait>(mFSMContext);
        }
    }
} // namespace mrover
