#pragma once

#include <algorithm>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/subscription.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <Eigen/Geometry>
// #include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/video/tracking.hpp>
#include <loop_profiler.hpp>
#include <mrover/msg/keyboard_yaw.hpp>
// Commented this out for now since it was messing with linter
// #include "constants.h"

#include <rclcpp/publisher.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <mrover/msg/ik.hpp>
#include "mrover/action/typing_deltas.hpp"
#include "mrover/action/typing_code.hpp"

// Other
#include <lie.hpp>
#include <manif/impl/se3/SE3.h>
#include <parameter.hpp>