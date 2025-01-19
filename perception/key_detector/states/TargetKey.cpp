#pragma once

#include "TargetKey.hpp"
#include <rclcpp/client.hpp>

namespace mrover{
TargetKey::TargetKey(const std::shared_ptr<FSMCtx> fsm_ctx) : fsm_ctx(fsm_ctx) 
{
        mIkTargetPub = fsm_ctx->node->create_publisher<msg::IK>("arm_ik", 1);
}

auto TargetKey::onLoop() -> State*{

    //while not cancelled and code_index < goal->get_goal()->code.size()
        // get the location of the key by querying model
        // use ik to move to the location
        // transition to press key state
    auto goal_handle = fsm_ctx->goal_handle;
    auto goal = goal_handle->get_goal();
    auto node = fsm_ctx->node;


    // if we have pressed all the keys, end
    if (fsm_ctx->curr_key_index > goal->code.size()){
        return nullptr;
    }
    

    if(goal_handle->is_canceling())
    {
        return StateMachine::make_state<Cancel>(fsm_ctx); //cancel if the goal is cancelled
    }

    auto client = node->create_client<srv::GetKeyLoc>("GetKeyLoc");

    auto request = std::make_shared<srv::GetKeyLoc::Request>();
    request->key = goal->code[fsm_ctx->curr_key_index];
    auto result = client->async_send_request(request);

    // Wait for the result. 
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto x = result.get()->x;
        auto y = result.get()->y;

        //move arm with ik
        msg::IK ik;
        ik.target.header.stamp = node->get_clock()->now();
        ik.target.header.frame_id = "arm_base_link"; // TODO: fix
        ik.target.pose.position.x = x;
        ik.target.pose.position.y = y;
        ik.target.pose.position.z = 0;
        mIkTargetPub->publish(ik);


        //verify that the arm is within the threshold
        bool within_threshold = false;
        if(within_threshold){
            fsm_ctx->curr_key_index++;
            return StateMachine::make_state<PressKey>(fsm_ctx);
        } else {
            return StateMachine::make_state<TargetKey>(fsm_ctx);
        }

    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }    


}
}