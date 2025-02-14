#pragma once

#include "TargetKey.hpp"
#include "key_detector/states/Cancel.hpp"
#include "lie.hpp"
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <rclcpp/client.hpp>

namespace mrover{
TargetKey::TargetKey(const std::shared_ptr<FSMCtx> fsm_ctx) : fsm_ctx(fsm_ctx), sleepRate(0.2)
{
}

auto TargetKey::onLoop() -> State*{

    //while not cancelled and code_index < goal->get_goal()->code.size()
        // get the location of the key by querying model
        // use ik to move to the location
        // transition to press key state

    auto goal_handle = fsm_ctx->goal_handle;

    if (goal_handle == nullptr)
    {
        //Improper intialization, immediate stop should be initiated
        return StateMachine::make_state<Cancel>(fsm_ctx);
    }

    auto goal = goal_handle->get_goal();
    auto node = fsm_ctx->node;

    // if we have pressed all the keys, end

    if (fsm_ctx->curr_key_index >= goal->code.size()){
        //We're done with the current sequence, so we move to an off position to prevent segfault
        fsm_ctx->init = false;
        return StateMachine::make_state<Off>(fsm_ctx); 
    }

    if(goal_handle->is_canceling())
    {
        return StateMachine::make_state<Cancel>(fsm_ctx); //cancel if the goal is cancelled
    }

    geometry_msgs::msg::Vector3 ik;

    //Allows for two seperate paths, simulator for running in simulation, and the bottom for runnin in normal
    if(goal->simulator)
    {
        auto key_loc = mrover::SE3Conversions::fromTfTree(*fsm_ctx->mTfBuffer, std::format("{}_key_truth", goal->code[fsm_ctx->curr_key_index]), "arm_e_link");
        ik.x = key_loc.x();
        ik.y = key_loc.y();
        ik.z = key_loc.z();
    }
    else 
    {
        auto client = node->create_client<srv::GetKeyLoc>("GetKeyLoc");

        auto request = std::make_shared<srv::GetKeyLoc::Request>();
        request->key = goal->code[fsm_ctx->curr_key_index];
        auto result = client->async_send_request(request);
        
        //Wait 1 second for promise at most (vision model runs at around 20 hertz, so this gives ample time)
        if(result.wait_for(std::chrono::seconds(1)) == std::future_status::ready)
        {
            ik.x = result.get()->x;
            ik.y = result.get()->y;
        }
        else
        {
            //Timed out should cancel, as we assume the vision module is either dead or timed out
            return StateMachine::make_state<Cancel>(fsm_ctx);
        }
    }
    
    //move arm with ik

    //print what key is being targeted
    RCLCPP_INFO(node->get_logger(), "Targeting key: %s", std::string(1, goal->code[fsm_ctx->curr_key_index]).c_str());
    fsm_ctx->mIkTargetPub->publish(ik);

    // subscribe to arm state
    // dist
    double magnitude = 0.0;

    if(goal->simulator)
    {
        auto thresholdCheck = mrover::SE3Conversions::fromTfTree(*fsm_ctx->mTfBuffer, std::format("{}_key_truth", goal->code[fsm_ctx->curr_key_index]), "arm_e_link");
        magnitude = std::sqrt(thresholdCheck.x() * thresholdCheck.x() + thresholdCheck.y() * thresholdCheck.y() + thresholdCheck.z() * thresholdCheck.z());
    }
    else
    {
        auto client = node->create_client<srv::GetKeyLoc>("GetKeyLoc");

        auto request = std::make_shared<srv::GetKeyLoc::Request>();
        request->key = goal->code[fsm_ctx->curr_key_index];
        auto result = client->async_send_request(request);
        
        //Wait 1 second for promise at most (vision model runs at around 20 hertz, so this gives ample time)
        if(result.wait_for(std::chrono::seconds(1)) == std::future_status::ready)
        {
            magnitude = std::sqrt(result.get()->x * result.get()->x + result.get()->y * result.get()->y)
        }
        else
        {
            //Timed out should cancel, as we assume the vision module is either dead or timed out
            return StateMachine::make_state<Cancel>(fsm_ctx);
        }
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), std::format("Magnitude: {}", magnitude));
    if(magnitude < 0.1){
        fsm_ctx->curr_key_index++;
        return StateMachine::make_state<PressKey>(fsm_ctx);
    } else {
        return StateMachine::make_state<TargetKey>(fsm_ctx);
    }      
    //} else {
     //   std::cout << "Check Cancel\n";
     //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
     //   return StateMachine::make_state<TargetKey>(fsm_ctx);
    //}    


}
}