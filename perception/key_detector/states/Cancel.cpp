#include "Cancel.hpp"

namespace mrover {
    Cancel::Cancel(std::shared_ptr<FSMContext> const& fsmContext) : mFSMContext(fsmContext) {}

    auto Cancel::onLoop() -> State* {
        RCLCPP_INFO(mFSMContext->node->get_logger(), "Cancel Onloop");

        //set the velocity of arm to zero
        return this; //This should end loop
    }
} // namespace mrover
