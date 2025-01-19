#pragma once

#include "PressKey.hpp"

namespace mrover{
PressKey::PressKey(const std::shared_ptr<FSMData> fsm_data) : fsm_data(fsm_data) 
{

}

auto PressKey::onLoop() -> State*{

    //while not cancelled 
        // If not ESW Failed
            // query press key

    if(fsm_data->goal_handle->is_canceling())
    {
        return StateMachine::make_state<Cancel>(fsm_data); //cancel if the goal is cancelled
    }
    
}
}