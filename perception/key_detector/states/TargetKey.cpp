#include "TargetKey.hpp"

namespace mrover {
    TargetKey::TargetKey(std::shared_ptr<FSMContext> const& fsmContext) : mFSMContext(fsmContext), sleepRate(0.2) {}

    auto TargetKey::onLoop() -> State* {
        RCLCPP_INFO(mFSMContext->node->get_logger(), "Target Key onLoop");
        if(mFSMContext->goal_handle->is_canceling()){
            return StateMachine::make_state<Cancel>(mFSMContext);
        }
        char key = mFSMContext->goal_handle->get_goal()->code[mFSMContext->curr_key_index];
        RCLCPP_INFO_STREAM(mFSMContext->node->get_logger(), "FSM Targeting " << key);

        // CHANGE
        
        SE3d keyInArmFrame = SE3Conversions::fromTfTree(mFSMContext->buffer, "e_key_truth", "arm_e_link");
        RCLCPP_INFO_STREAM(mFSMContext->node->get_logger(), "Y Z in EE Frame " << keyInArmFrame.translation().y() << " " << keyInArmFrame.translation().z());
        return this;

        // CHANGE

        std::shared_ptr<action::KeyAction::Feedback> feedback = std::make_shared<action::KeyAction::Feedback>();
        feedback->key = key;
        feedback->state = "Target Key";
        mFSMContext->goal_handle->publish_feedback(feedback);

        return StateMachine::make_state<PressKey>(mFSMContext);
    }
} // namespace mrover
