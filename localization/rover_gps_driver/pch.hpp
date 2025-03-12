#pragma once

#include <string>
#include <iostream>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mrover/msg/fix_status.hpp>
#include <mrover/msg/heading.hpp>
#include <mrover/msg/fix_type.hpp>
#include <rtcm_msgs/msg/message.hpp>
#include <mrover/msg/satellite_signal.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
