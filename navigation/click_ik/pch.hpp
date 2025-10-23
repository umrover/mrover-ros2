#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <execution>
#include <limits>
#include <numeric>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>

#include <rclcpp/node.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <lie.hpp>
#include <point.hpp>


#include "../navigation/arm_controller/arm_controller.hpp"
#include "mrover/action/click_ik.hpp"
#include "mrover/srv/ik_mode.hpp"
#include "mrover/msg/ik.hpp"
