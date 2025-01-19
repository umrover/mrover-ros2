#pragma once

#include "TargetKey.hpp"
#include <rclcpp/client.hpp>

namespace mrover{
TargetKey::TargetKey(const std::shared_ptr<FSMData> fsm_data) : fsm_data(fsm_data) 
{
}

auto TargetKey::onLoop() -> State*{

    //while not cancelled and code_index < goal->get_goal()->code.size()
        // get the location of the key by querying model
        // use ik to move to the location
        // transition to press key state
    if(fsm_data->goal_handle->is_canceling())
    {
        return StateMachine::make_state<Cancel>(fsm_data); //cancel if the goal is cancelled
    }
    auto request = std::make_shared<srv::GetKeyLoc::Request>();
    auto client = fsm_data->client;

    auto result = client->async_send_request(request);

    // Wait for the result. 
    //This node likely needs to be the node the original client was created with
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto x = result.get()->x;
        auto y = result.get()->y;

        //Do arm stuff down here
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }
    

    


}
}