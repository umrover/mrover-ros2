#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/subscription.hpp>
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
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/video/tracking.hpp>
// Commented this out for now since it was messing with linter
// #include "constants.h"
