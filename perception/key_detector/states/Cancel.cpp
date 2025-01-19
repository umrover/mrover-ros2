#pragma once

#include "Cancel.hpp"

namespace mrover{
Cancel::Cancel(const std::shared_ptr<FSMData> fsm_data) : fsm_data(fsm_data) 
{

}

auto Cancel::onLoop() -> State*{

    //set the velocity of arm to zero
    return nullptr; //This should end loop
}
}