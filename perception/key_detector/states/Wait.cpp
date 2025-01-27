#include "Wait.hpp"
#include <rclcpp/utilities.hpp>

namespace mrover {

    Wait::Wait(const std::shared_ptr<FSMCtx> fsm_ctx) 
        : sleepRate(1.0), fsm_ctx(fsm_ctx) // Initialize sleepRate with a parameter (e.g., 1.0)
    {
    }

    auto Wait::onLoop() -> State* {

        while(rclcpp::ok()){
            auto logger = rclcpp::get_logger("Wait");
            RCLCPP_INFO_STREAM(logger, "Entered Wait " <<  "\n");
            sleepRate.sleep();
            RCLCPP_INFO_STREAM(logger, "Exiting Wait " <<  "\n");

            return StateMachine::make_state<TargetKey>(fsm_ctx);
        }

        // if breaks end state machine?
        return nullptr;
    }
}
