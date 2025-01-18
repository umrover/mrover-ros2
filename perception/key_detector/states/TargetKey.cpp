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


    auto request = std::make_shared<srv::GetKeyLoc::Request>();
    auto client = fsm_data->client;

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }
    
    


}
}