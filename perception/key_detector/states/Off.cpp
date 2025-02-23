#pragma once

#include "Off.hpp"

namespace mrover{
Off::Off(const std::shared_ptr<FSMCtx> fsm_ctx) : fsm_ctx(fsm_ctx) 
{

}

auto Off::onLoop() -> State*{

    //set the velocity of arm to zero

    if(fsm_ctx->init)
    {
        return StateMachine::make_state<TargetKey>(fsm_ctx);
    }

    return StateMachine::make_state<Off>(fsm_ctx);
}
}