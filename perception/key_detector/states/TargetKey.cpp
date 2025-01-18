#pragma once

#include "TargetKey.hpp"

namespace mrover{
TargetKey::TargetKey(const std::shared_ptr<FSMData> fsm_data) : fsm_data(fsm_data) 
{

}

auto TargetKey::onLoop() -> State*{

    //while not cancelled and code_index < goal->get_goal()->code.size()
        // get the location of the key by querying model
        // use ik to move to the location
        // transition to press key state


}
}