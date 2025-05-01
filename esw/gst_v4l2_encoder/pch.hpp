#pragma once

#include <atomic>
#include <format>
#include <thread>

#include <rclcpp/create_timer.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <mrover/srv/media_control.hpp>

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <libudev.h>

#include <opencv2/imgproc.hpp>

#include <websocket_server.hpp>

#include <gst_utils.hpp>
#include <parameter.hpp>
