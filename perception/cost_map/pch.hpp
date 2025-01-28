#pragma once

#include <algorithm>
#include <cstdint>
#include <execution>
#include <format>
#include <numbers>
#include <memory>

#include <Eigen/Core>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/service.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <lie.hpp>
// #include <mrover/MoveCostMap.h>
#include <point.hpp>
#include <parameter.hpp>
#include <mrover/srv/move_cost_map.hpp>