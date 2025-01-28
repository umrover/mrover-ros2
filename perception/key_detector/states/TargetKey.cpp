#pragma once

#include "TargetKey.hpp"
#include "lie.hpp"
#include <rclcpp/client.hpp>
#include <tf2_ros/buffer.h>

namespace mrover{
TargetKey::TargetKey(const std::shared_ptr<FSMCtx> fsm_ctx) : fsm_ctx(fsm_ctx), sleepRate(0.2)
{
}

auto TargetKey::onLoop() -> State*{
    RCLCPP_INFO(fsm_ctx->node->get_logger(), "TargetKey Onloop");

    //while not cancelled and code_index < goal->get_goal()->code.size()
        // get the location of the key by querying model
        // use ik to move to the location
        // transition to press key state
    
    if(mIkTargetPub == nullptr)
        mIkTargetPub = fsm_ctx->node->create_publisher<msg::IK>("arm_ik", 1);

    auto goal_handle = fsm_ctx->goal_handle;

    if (goal_handle == nullptr)
        return StateMachine::make_state<TargetKey>(fsm_ctx);

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

    //Replace With Service Start
    //arm_e_link
    
    /*
    auto buffer = tf2_ros::Buffer(node->get_clock());
    tf2_ros::TransformBroadcaster mTfBroadcaster{node};
    auto tf = mrover::SE3Conversions::fromTfTree(buffer, std::format("{}_truth", goal->code[fsm_ctx->curr_key_index]), "lander_truth", node->get_clock()->now());
    auto origin = mrover::SE3Conversions::fromTfTree(buffer, "map", "arm_e_link", node->get_clock()->now());

    auto target = SE3d(tf.translation());

    SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_e_link", "map", target, node->get_clock()->now());

    */
    //Replace with Service End
    /*
    auto client = node->create_client<srv::GetKeyLoc>("GetKeyLoc");

    auto request = std::make_shared<srv::GetKeyLoc::Request>();
    request->key = goal->code[fsm_ctx->curr_key_index];
    auto result = client->async_send_request(request);

    result.wait_for(std::chrono::seconds(1));
    
    // Wait for the result. 
    //if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), result) ==
     //   rclcpp::FutureReturnCode::SUCCESS)
    //{
    std::cout << "Success" << result.get()->x << "," << result.get()->y << '\n';
        int64_t x = result.get()->x;
        int64_t y = result.get()->y;

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
        sleepRate.sleep();

        // subscribe to arm state
        // dist



        if(within_threshold){
            fsm_ctx->curr_key_index++;
            return StateMachine::make_state<PressKey>(fsm_ctx);
        } else {
            return StateMachine::make_state<TargetKey>(fsm_ctx);
        }        */


    //} else {
     //   std::cout << "Check Cancel\n";
     //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
     //   return StateMachine::make_state<TargetKey>(fsm_ctx);
    //}    


}
}
