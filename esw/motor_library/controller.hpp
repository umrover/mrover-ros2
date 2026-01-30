#pragma once

#include "can_device.hpp"
#include "units.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <variant>

namespace mrover {

    template<typename Derived>
    class ControllerBase {
    public:
        ControllerBase(rclcpp::Node::SharedPtr node, std::string master_name, std::string controller_name)
            : m_node{std::move(node)},
              m_master_name{std::move(master_name)},
              m_controller_name{std::move(controller_name)},
              m_device{m_node, m_master_name, m_controller_name,
                       [this](can_msg_t const& msg) {
                           static_cast<Derived*>(this)->process_message(msg);
                       }} {}

        virtual ~ControllerBase() = default;

        // common state accessors
        [[nodiscard]] auto get_name() const -> std::string { return m_controller_name; }
        [[nodiscard]] auto get_state() const -> std::string { return m_state; }
        [[nodiscard]] auto get_error() const -> std::string { return m_error_state; }
        [[nodiscard]] auto get_position() const -> float { return m_position; }
        [[nodiscard]] auto get_velocity() const -> float { return m_velocity; }
        [[nodiscard]] auto get_current() const -> float { return m_current; }
        [[nodiscard]] auto getLimitsHitBits() const -> std::uint8_t {
            std::uint8_t limits_hit{};
            for (int i = 0; i < 4; ++i) {
                limits_hit |= m_limit_hit.at(i) << i;
            }
            return limits_hit;
        }

    protected:
        rclcpp::Node::SharedPtr m_node;
        std::string m_master_name, m_controller_name;
        CanDevice m_device;

        // state variables
        std::string m_state{"Stopped"};
        std::string m_error_state{"None"};
        float m_position{0.0f};
        float m_velocity{0.0f};
        float m_current{0.0f};
        std::array<bool, 4> m_limit_hit{};
    };

} // namespace mrover
