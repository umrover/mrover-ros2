#pragma once

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <format>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <mrover/msg/camera_controls.hpp>
#include <mrover/srv/media_control.hpp>
#include <mrover/srv/set_camera_controls.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <libudev.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <gst_utils.hpp>
#include <parameter.hpp>
