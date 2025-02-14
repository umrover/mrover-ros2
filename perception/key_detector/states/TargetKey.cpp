#pragma once

#include "TargetKey.hpp"
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
        fsm_ctx->init = false;
        return StateMachine::make_state<Off>(fsm_ctx);
    }
 

    auto goal = goal_handle->get_goal();
    auto node = fsm_ctx->node;

    // if we have pressed all the keys, end

    if (fsm_ctx->curr_key_index >= goal->code.size()){
        fsm_ctx->init = false;
        return StateMachine::make_state<Off>(fsm_ctx);
    }

    if(goal_handle->is_canceling())
    {
        return StateMachine::make_state<Cancel>(fsm_ctx); //cancel if the goal is cancelled
    }

    //Replace With Service Start
    //arm_e_link

    auto key_loc = mrover::SE3Conversions::fromTfTree(*fsm_ctx->mTfBuffer, std::format("{}_key_truth", goal->code[fsm_ctx->curr_key_index]), "arm_e_link");

    // auto key_loc_test = mrover::SE3Conversions::fromTfTree(buffer, "base_link", "q_key_truth" , node->get_clock()->now());

    //auto relative_rotation = origin.rotation().transpose() * key_loc.rotation();
    //auto target = SE3d(key_loc.translation() - origin.translation(), SO3d(relative_rotation));

    //Replace with Service End
    /*
    auto client = node->create_client<srv::GetKeyLoc>("GetKeyLoc");

    auto request = std::make_shared<srv::GetKeyLoc::Request>();
    request->key = goal->code[fsm_ctx->curr_key_index];
    auto result = client->async_send_request(request);

    result.wait_for(std::chrono::seconds(1));
      */
    // Wait for the result. 
    //if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), result) ==
     //   rclcpp::FutureReturnCode::SUCCESS)
    //{

    //move arm with ik
    geometry_msgs::msg::Vector3 ik;
    ik.x = key_loc.x();
    ik.y = key_loc.y();
    ik.z = key_loc.z();

    //print what key is being targeted
    RCLCPP_INFO(node->get_logger(), "Targeting key: %s", std::string(1, goal->code[fsm_ctx->curr_key_index]).c_str());
    fsm_ctx->mIkTargetPub->publish(ik);


    //verify that the arm is within the threshold
    bool within_threshold = false;
    // sleepRate.sleep();

    // subscribe to arm state
    // dist

    auto thresholdCheck = mrover::SE3Conversions::fromTfTree(*fsm_ctx->mTfBuffer, std::format("{}_key_truth", goal->code[fsm_ctx->curr_key_index]), "arm_e_link");

    double magnitude = std::sqrt(thresholdCheck.x() * thresholdCheck.x() + thresholdCheck.y() * thresholdCheck.y() + thresholdCheck.z() * thresholdCheck.z());

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