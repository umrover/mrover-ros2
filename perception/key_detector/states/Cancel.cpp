#pragma once

#include "Cancel.hpp"

namespace mrover{
Cancel::Cancel(const std::shared_ptr<FSMCtx> fsm_ctx) : fsm_ctx(fsm_ctx) 
{

}

auto Cancel::onLoop() -> State*{

    fsm_ctx->init = false;

    //set the velocity of arm to zero
    return StateMachine::make_state<Off>(fsm_ctx); // Sends to off state
}
}