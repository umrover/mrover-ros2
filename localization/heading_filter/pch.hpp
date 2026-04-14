#pragma once

// 1. Standard C++ libraries
#include <functional>
#include <chrono>
#include <cstdint>
#include <memory>

// 2. Third-party geometry/math libraries
#include <lie.hpp>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>

// 3. ROS 2 core systems
#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl/time.h>

// 4. ROS 2 utilities
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters//synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// 5. Standard ROS 2 message types
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

// 6. MRover-specific custom messages
#include <mrover/msg/heading.hpp>
#include <mrover/msg/fix_type.hpp>
#include <mrover/msg/fix_status.hpp>

// 7. MRover-specific local headers
#include <parameter.hpp>


