#include "PressKey.hpp"

namespace mrover {
    PressKey::PressKey(std::shared_ptr<FSMContext> const& fsmContext) : mFSMContext(fsmContext), mToleranceHitcount{0}, mState{PRESS_KEY_STATE::EXTEND} {
        
    }

    auto PressKey::onLoop() -> State* {
        RCLCPP_INFO(mFSMContext->node->get_logger(), "Press Key onLoop");
        if(mFSMContext->goal_handle->is_canceling()){
            return StateMachine::make_state<Cancel>(mFSMContext);
        }


        // Get the current key to be targeted
        char key = mFSMContext->goal_handle->get_goal()->code[mFSMContext->curr_key_index];

        switch(mState){
            case PRESS_KEY_STATE::EXTEND:{
                // Check to see if the end effector has been extended long enough
                if(mToleranceHitcount == MAX_KEY_HITCOUNT){
                    mState = PRESS_KEY_STATE::RETRACT;
                    mToleranceHitcount = 0;
                }

                // Publish feedback
                std::shared_ptr<action::KeyAction::Feedback> feedback = std::make_shared<action::KeyAction::Feedback>();
                feedback->key = key;
                feedback->state = "Press Key Press";
                mFSMContext->goal_handle->publish_feedback(feedback);

                // Generate the vector
                SE3d keyInArmFrame = SE3Conversions::fromTfTree(mFSMContext->buffer, std::format("{}_key_truth", key), "arm_e_link");
                geometry_msgs::msg::Vector3 vec;
                vec.x = keyInArmFrame.translation().x();
                vec.y = keyInArmFrame.translation().y();
                vec.z = keyInArmFrame.translation().z();
                mFSMContext->armVelocityPub->publish(vec);

                // Update hitcount
                double distance = std::abs(keyInArmFrame.translation().x());

                RCLCPP_INFO_STREAM(mFSMContext->node->get_logger(), "X Distance " << keyInArmFrame.translation().x() << " distance " << distance);

                if(distance < KEY_TOLERANCE){
                    ++mToleranceHitcount;
                }else{
                    --mToleranceHitcount;
                }

                mToleranceHitcount = std::max(0, std::min(mToleranceHitcount, MAX_KEY_HITCOUNT));
                break;
            }
            case PRESS_KEY_STATE::RETRACT:{
                // Check to see if the end effector has been extended long enough
                if(mToleranceHitcount == MAX_KEY_HITCOUNT){
                    ++mFSMContext->curr_key_index;
                    return StateMachine::make_state<Wait>(mFSMContext);
                }

                // Publish feedback
                std::shared_ptr<action::KeyAction::Feedback> feedback = std::make_shared<action::KeyAction::Feedback>();
                feedback->key = key;
                feedback->state = "Press Key Retract";
                mFSMContext->goal_handle->publish_feedback(feedback);

                // Generate the vector
                SE3d keyInArmFrame = SE3Conversions::fromTfTree(mFSMContext->buffer, std::format("{}_key_truth", key), "arm_e_link");
                geometry_msgs::msg::Vector3 vec;
                vec.x = keyInArmFrame.translation().x() - KEYBOARD_TYPING_HEIGHT; // This will cause the arm to return to KEYBOARD_TYPING_HEIGHT
                vec.y = keyInArmFrame.translation().y();
                vec.z = keyInArmFrame.translation().z();
                mFSMContext->armVelocityPub->publish(vec);

                // Update hitcount
                double distance = keyInArmFrame.translation().x() - KEYBOARD_TYPING_HEIGHT;

                RCLCPP_INFO_STREAM(mFSMContext->node->get_logger(), "X Distance " << keyInArmFrame.translation().x() << " distance " << distance);

                if(distance < KEY_TOLERANCE){
                    ++mToleranceHitcount;
                }else{
                    --mToleranceHitcount;
                }

                mToleranceHitcount = std::max(0, std::min(mToleranceHitcount, MAX_KEY_HITCOUNT));
                break;
            }
            default:{
                break;
            }
        }

    


        return this;
    }
} // namespace mrover
