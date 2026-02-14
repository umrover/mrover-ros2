#pragma once

#include "pch.hpp"

namespace mrover {

    struct CanNetLink {

        CanNetLink() = default;

        CanNetLink(rclcpp::Logger const& logger, std::string);

        CanNetLink(CanNetLink const&) = delete;
        auto operator=(CanNetLink const&) -> CanNetLink& = delete;

        CanNetLink(CanNetLink&& other) noexcept {
            *this = std::move(other);
        }

        auto operator=(CanNetLink&& other) noexcept -> CanNetLink& {
            m_interface = std::move(other.m_interface);
            m_socket = std::exchange(other.m_socket, nullptr);
            m_cache = std::exchange(other.m_cache, nullptr);
            return *this;
        }

        ~CanNetLink();

        std::string m_interface{};
        nl_cache* m_cache{};
        nl_sock* m_socket{};
    };

} // namespace mrover
