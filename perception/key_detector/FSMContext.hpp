#pragma once
#include <rclcpp/rclcpp.hpp>
#include "mrover/action/key_action.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"

using KeyAction = mrover::action::KeyAction;
using GoalHandleKeyAction = rclcpp_action::ServerGoalHandle<KeyAction>;

namespace mrover {
    struct FSMContext {
        std::shared_ptr<GoalHandleKeyAction> goal_handle;
        std::shared_ptr<rclcpp::Node> node;
        tf2_ros::Buffer& buffer;
        int curr_key_index;
        bool restart;

        explicit FSMContext(tf2_ros::Buffer& buf) : goal_handle{nullptr}, node{nullptr}, buffer{buf}, curr_key_index{0}, restart{false}{}
    };
} // namespace mrover
