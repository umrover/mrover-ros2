#include <iostream>
#include <memory>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mrover/msg/heading.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <lie.hpp>
#include <manif/SO3.h>
#include <manif/SE_2_3.h>
