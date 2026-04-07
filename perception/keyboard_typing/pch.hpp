#pragma once

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>

#include <Eigen/Geometry>
#include <Eigen/src/Geometry/Quaternion.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/executors.hpp>


#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <mrover/msg/ik.hpp>
#include <mrover/msg/keyboard_yaw.hpp>
#include <mrover/msg/position.hpp>

#include "mrover/action/typing_code.hpp"
#include "mrover/action/typing_deltas.hpp"

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <manif/impl/se3/SE3.h>
#include <lie.hpp>

#include <loop_profiler.hpp>
#include <parameter.hpp>
#include "constants.hpp"
#include "keyboard_typing/constants.hpp"
#include <mrover/srv/ik_mode.hpp>

#include "mrover/msg/detail/position__struct.hpp"
#include "mrover/srv/detail/pusher__struct.hpp"