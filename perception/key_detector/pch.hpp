#pragma once

// Ros Client Library
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "mrover/msg/detail/position__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// STL
#include <functional>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>

// MRover Libs
#include <state_machine.hpp>
#include <state.hpp>
#include <state_publisher_server.hpp>

//Actions
#include <mrover/action/key_action.hpp>

//Services
#include <mrover/srv/get_key_loc.hpp>

//Messages
#include <mrover/msg/ik.hpp>

//Lie
#include <lie.hpp>

using KeyAction = mrover::action::KeyAction;
using GoalHandleKeyAction = rclcpp_action::ServerGoalHandle<KeyAction>;

struct FSMCtx{
  std::shared_ptr<GoalHandleKeyAction> goal_handle;
  std::shared_ptr<rclcpp::Node> node;
  int curr_key_index;
  std::shared_ptr<tf2_ros::Buffer> mTfBuffer;
  rclcpp::Publisher<mrover::msg::IK>::SharedPtr mIkTargetPub;
  mrover::msg::Position::SharedPtr mIKPos;
};

// States
#include "states/Cancel.hpp"
#include "states/PressKey.hpp"
#include "states/Wait.hpp"
#include "states/TargetKey.hpp"
#include "states/PressKey.hpp"
