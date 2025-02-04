#include "TargetKey.hpp"

namespace mrover {
    TargetKey::TargetKey(std::shared_ptr<FSMContext> const& fsmContext) : mFSMContext(fsmContext), sleepRate(0.2), mToleranceHitcount{0} {}

    auto TargetKey::onLoop() -> State* {
        RCLCPP_INFO(mFSMContext->node->get_logger(), "Target Key onLoop");
        if(mFSMContext->goal_handle->is_canceling()){
            return StateMachine::make_state<Cancel>(mFSMContext);
        }
        char key = mFSMContext->goal_handle->get_goal()->code[mFSMContext->curr_key_index];
        RCLCPP_INFO_STREAM(mFSMContext->node->get_logger(), "FSM Targeting " << key);

        // CHANGE
        

        // Publish feedback
        std::shared_ptr<action::KeyAction::Feedback> feedback = std::make_shared<action::KeyAction::Feedback>();
        feedback->key = key;
        feedback->state = "Target Key";
        mFSMContext->goal_handle->publish_feedback(feedback);

        // Generate the vector
        SE3d keyInArmFrame = SE3Conversions::fromTfTree(mFSMContext->buffer, std::format("{}_key_truth", key), "arm_e_link");
        geometry_msgs::msg::Vector3 vec;
        vec.y = keyInArmFrame.translation().y();
        vec.z = keyInArmFrame.translation().z();
        mFSMContext->armVelocityPub->publish(vec);

        // Update hitcount
        double distance = std::sqrt(std::pow(vec.y, 2) + std::pow(vec.z, 2));

        RCLCPP_INFO_STREAM(mFSMContext->node->get_logger(), "Y Z in EE Frame " << keyInArmFrame.translation().y() << " " << keyInArmFrame.translation().z() << " distance " << distance);

        if(distance < KEY_TOLERANCE){
            ++mToleranceHitcount;
        }else{
            --mToleranceHitcount;
        }

        mToleranceHitcount = std::max(0, std::min(mToleranceHitcount, MAX_KEY_HITCOUNT));

        if(mToleranceHitcount == MAX_KEY_HITCOUNT){
            return StateMachine::make_state<PressKey>(mFSMContext);
        }

        return this;
    }
} // namespace mrover
