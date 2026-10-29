#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/executors.hpp>

// #include <rclcpp_action/client.hpp>
// #include <rclcpp_action/create_client.hpp>
// #include <rclcpp_action/client_goal_handle.hpp>
// #include <rclcpp_action/server.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <numbers>

#include "loop_profiler.hpp"
#include "parameter.hpp"

#include <lie.hpp>

#include <mrover/msg/body.hpp>