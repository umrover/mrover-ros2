#pragma once

#include "PressKey.hpp"

namespace mrover{
PressKey::PressKey(const std::shared_ptr<FSMCtx> fsm_ctx) : fsm_ctx(fsm_ctx) 
{

}

auto PressKey::onLoop() -> State*{

    //while not cancelled 
        // If not ESW Failed
            // query press key
    auto goal_handle = fsm_ctx->goal_handle;
    auto goal = goal_handle->get_goal();


    if(fsm_ctx->goal_handle->is_canceling())
    {
        return StateMachine::make_state<Cancel>(fsm_ctx); //cancel if the goal is cancelled
    }

    auto key_loc = mrover::SE3Conversions::fromTfTree(*fsm_ctx->mTfBuffer, std::format("{}_key_truth", goal->code[fsm_ctx->curr_key_index]), "arm_e_link");

    geometry_msgs::msg::Vector3 press;
    press.x = key_loc.x() - 0.5;
    press.y = key_loc.y();
    press.z = key_loc.z();

    fsm_ctx->mIkTargetPub->publish(press);

    geometry_msgs::msg::Vector3 unpress;
    unpress.x = key_loc.x() + 0.5;
    unpress.y = key_loc.y();
    unpress.z = key_loc.z();

    fsm_ctx->mIkTargetPub->publish(unpress);


    fsm_ctx->curr_key_index++;

    return StateMachine::make_state<Wait>(fsm_ctx);
    
}
}