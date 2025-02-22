#include "Wait.hpp"
#include <rclcpp/utilities.hpp>

namespace mrover {

    Wait::Wait(std::shared_ptr<FSMContext> const& fsmContext)
        : sleepRate(0.1), mFSMContext{fsmContext} {
    }

    auto Wait::onLoop() -> State* {
        RCLCPP_INFO(mFSMContext->node->get_logger(), "Wait onLoop");

        char key = mFSMContext->goal_handle->get_goal()->code[mFSMContext->curr_key_index];

        if(mFSMContext->goal_handle->is_canceling()){
            return StateMachine::make_state<Cancel>(mFSMContext);
        }

        // Wait while publishing no movement
        auto begin = mFSMContext->node->get_clock()->now().seconds();
        while(mFSMContext->node->get_clock()->now().seconds() < begin + WAIT_DURATION){
            try{
                SE3d keyInArmFrame = SE3Conversions::fromTfTree(mFSMContext->buffer, std::format("{}_key_truth", key), "arm_e_link");
                geometry_msgs::msg::Vector3 vec;
                vec.x = keyInArmFrame.translation().x() - KEYBOARD_TYPING_HEIGHT; // This will cause the arm to return to KEYBOARD_TYPING_HEIGHT
                vec.y = keyInArmFrame.translation().y() - FINGER_OFFSET;
                vec.z = keyInArmFrame.translation().z();

                vec.x *= SPEED;
                vec.y *= SPEED;
                vec.z *= SPEED;

                mFSMContext->armVelocityPub->publish(vec);
            }catch(...){}
        }

        return StateMachine::make_state<TargetKey>(mFSMContext);
    }
} // namespace mrover
