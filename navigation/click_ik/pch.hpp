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

#include "rclcpp_action/rclcpp_action.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <lie.hpp>
#include <loop_profiler.hpp>
#include <point.hpp>


#include "../teleoperation/arm_controller/arm_controller.hpp"
#include "mrover/ClickIkAction.h"
#include "mrover/ClickIkGoal.h"
#include "mrover/IK.h"
