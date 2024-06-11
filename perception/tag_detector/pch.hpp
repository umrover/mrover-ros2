#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <execution>
#include <format>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>
#include <span>
#include <string>
#include <type_traits>
#include <unordered_map>

#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <mrover/msg/image_target.hpp>
#include <mrover/msg/image_targets.hpp>

#include <lie.hpp>
#include <loop_profiler.hpp>
#include <point.hpp>
