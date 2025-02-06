#pragma once

#include <algorithm>
#include <cstdint>
#include <execution>
#include <format>
#include <memory>
#include <numeric>
#include <random>
#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>
#include <unsupported/Eigen/EulerAngles>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <lie.hpp>
// #include <mrover/MoveCostMap.h>
#include <mrover/srv/move_cost_map.hpp>
#include <parameter.hpp>
#include <point.hpp>
#include <rclcpp/logging.hpp>
