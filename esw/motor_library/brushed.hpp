#pragma once

#include "controller.hpp"

namespace mrover {

    class BrushedController final : public ControllerBase<BrushedController> {
        enum struct mode_t : uint8_t {
            STOPPED = 0,
            FAULT = 1,
            THROTTLE = 5,
            POSITION = 6,
            VELOCITY = 7,
        };

        static auto byte2mode(uint8_t const value) -> mode_t {
            switch (value) {
                case 0:
                    return mode_t::STOPPED;
                case 1:
                    return mode_t::FAULT;
                case 5:
                    return mode_t::THROTTLE;
                case 6:
                    return mode_t::POSITION;
                case 7:
                    return mode_t::VELOCITY;
                default:
                    throw std::invalid_argument("invalid mode");
            }
        }

        mode_t m_reported_mode{mode_t::STOPPED};

        void ensure_mode(mode_t const target_mode) {
            if (m_reported_mode != target_mode) {
                m_device.publish_message(BMCModeCmd{static_cast<uint8_t>(target_mode), 1});
            }
        }

    public:
        using ControllerBase::ControllerBase;

        auto set_desired_throttle(Percent const throttle) -> void {
            ensure_mode(mode_t::THROTTLE);
            m_device.publish_message(BMCTargetCmd{throttle.get(), 1});
        }

        auto set_desired_position(Radians const position) -> void {
            ensure_mode(mode_t::POSITION);
            m_device.publish_message(BMCTargetCmd{position.get(), 1});
        }

        auto set_desired_velocity(RadiansPerSecond const velocity) -> void {
            ensure_mode(mode_t::VELOCITY);
            m_device.publish_message(BMCTargetCmd{velocity.get(), 1});
        }

        void process_message(can_msg_t const& msg) {
            std::visit([this](auto const& decoded) -> auto {
                using T = std::decay_t<decltype(decoded)>;

                if constexpr (std::is_same_v<T, BMCMotorState>) {
                    this->m_position = decoded.position;
                    this->m_velocity = decoded.velocity;
                    this->m_current = decoded.current;
                    this->m_limit_hit[0] = static_cast<bool>(decoded.limit_a);
                    this->m_limit_hit[1] = static_cast<bool>(decoded.limit_b);

                    // update internal bmc state
                    auto mode = byte2mode(decoded.mode);
                    this->m_reported_mode = mode;
                    this->m_state = (mode == mode_t::FAULT) ? "Fault" : "Running";
                } else if constexpr (std::is_same_v<T, BMCAck>) {
                    RCLCPP_DEBUG(m_node->get_logger(), "BMC Ack received: %u", decoded.data);
                }
            },
                       msg);
        }
    };
} // namespace mrover
