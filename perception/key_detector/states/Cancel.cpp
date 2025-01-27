#pragma once

#include "Cancel.hpp"

namespace mrover{
Cancel::Cancel(const std::shared_ptr<FSMCtx> fsm_ctx) : fsm_ctx(fsm_ctx) 
{

}

auto Cancel::onLoop() -> State*{

    //set the velocity of arm to zero
    return nullptr; //This should end loop
}
}