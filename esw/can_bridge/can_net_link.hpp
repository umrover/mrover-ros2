#pragma once

#include "pch.hpp"

namespace mrover {

    struct CANNetLink {

        CANNetLink() = default;

        CANNetLink(rclcpp::Logger const& logger, std::string);

        CANNetLink(CANNetLink const&) = delete;
        auto operator=(CANNetLink const&) -> CANNetLink& = delete;

        CANNetLink(CANNetLink&& other) noexcept {
            *this = std::move(other);
        }

        auto operator=(CANNetLink&& other) noexcept -> CANNetLink& {
            mInterface = std::move(other.mInterface);
            mSocket = std::exchange(other.mSocket, nullptr);
            mCache = std::exchange(other.mCache, nullptr);
            return *this;
        }

        ~CANNetLink();

        std::string mInterface{};
        nl_cache* mCache{};
        nl_sock* mSocket{};
    };

} // namespace mrover
