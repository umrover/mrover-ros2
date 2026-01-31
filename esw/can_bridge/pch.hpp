#pragma once

#include <bit>
#include <bitset>
#include <cctype>
#include <cstdint>
#include <format>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

#include <boost/asio/error.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/posix/basic_stream_descriptor.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/bimap.hpp>
#include <boost/bimap/unordered_set_of.hpp>

#include <boost/system/error_code.hpp>

#include <netlink/route/link.h>
#include <netlink/route/link/can.h>

#include <mrover/msg/can.hpp>
#include <CANBus1.hpp>
