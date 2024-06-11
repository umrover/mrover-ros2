#pragma once

#include "pch.hpp"

namespace mrover {

    struct CanNetLink {

        CanNetLink() = default;

        CanNetLink(rclcpp::Logger logger, std::string);

        CanNetLink(CanNetLink const&) = delete;
        auto operator=(CanNetLink const&) -> CanNetLink& = delete;

        CanNetLink(CanNetLink&& other) noexcept {
            *this = std::move(other);
        }

        auto operator=(CanNetLink&& other) noexcept -> CanNetLink& {
            mInterface = std::move(other.mInterface);
            mSocket = std::exchange(other.mSocket, nullptr);
            mCache = std::exchange(other.mCache, nullptr);
            return *this;
        }

        ~CanNetLink();

        std::string mInterface{};
        nl_cache* mCache{};
        nl_sock* mSocket{};
    };

} // namespace mrover
