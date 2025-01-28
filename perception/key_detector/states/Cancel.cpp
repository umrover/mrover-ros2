#pragma once

#include "Cancel.hpp"

namespace mrover{
Cancel::Cancel(const std::shared_ptr<FSMCtx> fsm_ctx) : fsm_ctx(fsm_ctx) 
{

}

auto Cancel::onLoop() -> State*{
    RCLCPP_INFO(fsm_ctx->node->get_logger(), "Cancel Onloop");

    //set the velocity of arm to zero
    return this; //This should end loop
}
}
