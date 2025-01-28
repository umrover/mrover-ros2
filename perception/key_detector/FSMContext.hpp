#pragma once
#include <rclcpp/rclcpp.hpp>
#include "mrover/action/key_action.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using KeyAction = mrover::action::KeyAction;
using GoalHandleKeyAction = rclcpp_action::ServerGoalHandle<KeyAction>;

namespace mrover {
    struct FSMContext {
        std::shared_ptr<GoalHandleKeyAction> goal_handle;
        std::shared_ptr<rclcpp::Node> node;
        int curr_key_index;
    };
} // namespace mrover
