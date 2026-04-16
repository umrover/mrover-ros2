#pragma once

// Precompiled headers should generally be included first
#include "keyboard_typing/pch.hpp"

// Standard Library
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <mrover/action/typing_code.hpp>
#include <mrover/action/typing_deltas.hpp>
#include <mrover/action/typing_position.hpp>
#include <mrover/msg/ik.hpp>
#include <mrover/msg/keyboard_yaw.hpp>
#include <mrover/msg/position.hpp>
#include <mrover/srv/ik_mode.hpp>
#include <mrover/srv/pusher.hpp>

#include <manif/impl/se3/SE3.h>

#include "constants.hpp"
#include "keyboard_typing/constants.hpp"
#include "lie.hpp"
#include <loop_profiler.hpp>
#include <parameter.hpp>