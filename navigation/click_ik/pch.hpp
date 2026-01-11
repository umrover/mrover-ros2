#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <execution>
#include <functional>
#include <limits>
#include <numeric>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>

#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <lie.hpp>
#include <point.hpp>


#include "../navigation/arm_controller/arm_controller.hpp"
#include "mrover/action/click_ik.hpp"
#include "mrover/action/detail/click_ik__struct.hpp"
#include "mrover/msg/ik.hpp"
#include "mrover/srv/ik_mode.hpp"
#include <mrover/msg/arm_status.hpp>
#include <mrover/msg/ik.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/types.hpp>
#include <tf2_ros/transform_listener.h>