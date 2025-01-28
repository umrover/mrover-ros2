#pragma once

#include "PressKey.hpp"

namespace mrover{
PressKey::PressKey(const std::shared_ptr<FSMCtx> fsm_ctx) : fsm_ctx(fsm_ctx) 
{

}

auto PressKey::onLoop() -> State*{
    RCLCPP_INFO(fsm_ctx->node->get_logger(), "PressKey Onloop");

    //while not cancelled 
        // If not ESW Failed
            // query press key

    if(fsm_ctx->goal_handle->is_canceling())
    {
        return StateMachine::make_state<Cancel>(fsm_ctx); //cancel if the goal is cancelled
    }
    
}
}
