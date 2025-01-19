#include "Wait.hpp"
#include <rclcpp/utilities.hpp>

namespace mrover {

    Wait::Wait(const std::shared_ptr<FSMCtx> fsm_data) 
        : sleepRate(1.0), fsm_data(fsm_data) // Initialize sleepRate with a parameter (e.g., 1.0)
    {
    }

    auto Wait::onLoop() -> State* {
        while(rclcpp::ok()){
            auto logger = rclcpp::get_logger("Wait");
            RCLCPP_INFO_STREAM(logger, "Entered Wait " <<  "\n");
            sleepRate.sleep();
            RCLCPP_INFO_STREAM(logger, "Exiting Wait " <<  "\n");

            // after wait this state machine should end 
            return StateMachine::make_state<Cancel>(fsm_data);
        }

        // compile fix
        return StateMachine::make_state<Cancel>(fsm_data);
    }
}
