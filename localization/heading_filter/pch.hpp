#pragma once

#include <functional>

#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters//synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <mrover/msg/heading.hpp>
#include <mrover/msg/fix_type.hpp>
#include <mrover/msg/fix_status.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <lie.hpp>