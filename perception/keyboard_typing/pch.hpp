#pragma once

#include <algorithm>

#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <Eigen/Geometry>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
// #include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <loop_profiler.hpp>
#include <mrover/msg/keyboard_yaw.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/video/tracking.hpp>
// Commented this out for now since it was messing with linter
// #include "constants.h"

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>

#include "mrover/action/typing_code.hpp"
#include "mrover/action/typing_deltas.hpp"
#include <mrover/msg/ik.hpp>

// Other
#include <lie.hpp>
#include <manif/impl/se3/SE3.h>
#include <parameter.hpp>

// Random and unordered, potential duplicates
#include "keyboard_typing/constants.h"
#include "lie.hpp"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <future>
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>
#include <mrover/msg/ik.hpp>
#include <mutex>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/server.hpp>
#include <thread>
#include <unordered_map>