#pragma once

#include <string>
#include <vector>
#include <array>
#include <deque>

#include <rclcpp/node.hpp>

#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <opencv4/opencv2/dnn.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <mrover/msg/object_bounding_box.hpp>
#include <mrover/msg/object_bounding_boxes.hpp>

#include <mrover/msg/tag_bounding_box.hpp>
#include <mrover/msg/tag_bounding_boxes.hpp>

#include <parameter.hpp>
