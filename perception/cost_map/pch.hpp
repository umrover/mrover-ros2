#pragma once

// STL
#include <algorithm>
#include <cstdint>
#include <execution>
#include <format>
#include <memory>
#include <numeric>
#include <random>
#include <cstddef>
#include <vector>
#include <utility>
#include <cstdlib>

// Eigen
#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>
#include <unsupported/Eigen/EulerAngles>

// ROS2 Core
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// Other
#include <lie.hpp>
#include <parameter.hpp>
#include <point.hpp>
#include <rclcpp/logging.hpp>

// Message types
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// Services
#include <rclcpp/service.hpp>
#include<mrover/srv/dilate_cost_map.hpp>
#include <mrover/srv/move_cost_map.hpp>