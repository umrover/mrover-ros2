#pragma once

#include "controller.hpp"

namespace mrover {

    template<IsUnit TOutputPosition>
    class BrushedController final : public ControllerBase<BrushedController<TOutputPosition>> {
    public:
        using OutputPosition = TOutputPosition;
        using OutputVelocity = compound_unit<OutputPosition, inverse<Seconds>>;

    private:
        using Base = ControllerBase<BrushedController>;

        using Base::m_controller_name;
        using Base::m_current;
        using Base::m_device;
        using Base::m_error_state;
        using Base::m_limit_hit;
        using Base::m_master_name;
        using Base::m_node;
        using Base::m_position;
        using Base::m_state;
        using Base::m_velocity;

        enum struct mode_t : uint8_t {
            STOPPED = 0,
            FAULT = 1,
            THROTTLE = 5,
            POSITION = 6,
            VELOCITY = 7,
        };

        enum struct bmc_error_t : uint8_t {
            NONE,                           // no error
            NO_MODE,                        // no mode selected
            INVALID_CONFIGURATION_FOR_MODE, // in position or velocity mode without feedback mechanism configured to close the loop
            INVALID_FLASH_CONFIG,           // signal that there are too many encoders connected or something
            WWDG_EXPIRED,                   // watchdog expired
            UNCALIBRATED,                   // received position or velocity mode before calibrated
            CAN_ERROR_FATAL,                // unrecoverable CAN error encountered
            I2C_ERROR_FATAL,                // unrecoverable I2C error encountered
            SPI_ERROR_FATAL,                // unrecoverable SPI error encountered
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

        static auto err2str(bmc_error_t const error) -> std::string {
            switch (error) {
                case bmc_error_t::NONE:
                    return "None";
                case bmc_error_t::NO_MODE:
                    return "No mode selected";
                case bmc_error_t::INVALID_CONFIGURATION_FOR_MODE:
                    return "Invalid configuration for mode";
                case bmc_error_t::INVALID_FLASH_CONFIG:
                    return "Invalid flash configuration";
                case bmc_error_t::WWDG_EXPIRED:
                    return "CAN watchdog expired";
                case bmc_error_t::UNCALIBRATED:
                    return "Uncalibrated";
                case bmc_error_t::CAN_ERROR_FATAL:
                    return "Fatal CAN error";
                case bmc_error_t::I2C_ERROR_FATAL:
                    return "Fatal I2C error";
                case bmc_error_t::SPI_ERROR_FATAL:
                    return "Fatal SPI error";
                default:
                    return "Unknown Error";
            }
        }

        mode_t m_reported_mode{mode_t::STOPPED};

        void ensure_mode(mode_t const target_mode) {
            if (m_reported_mode != target_mode) {
                m_device.publish_message(BMCModeCmd{static_cast<uint8_t>(target_mode), 1});
            }
        }

    public:
        BrushedController(rclcpp::Node::SharedPtr node, std::string master_name, std::string controller_name)
            : Base{std::move(node), std::move(master_name), std::move(controller_name)} {
        }

        auto set_desired_throttle(Percent const throttle) -> void {
            ensure_mode(mode_t::THROTTLE);
            m_device.publish_message(BMCTargetCmd{throttle.get(), 1});
        }

        auto set_desired_position(OutputPosition const position) -> void {
            ensure_mode(mode_t::POSITION);
            m_device.publish_message(BMCTargetCmd{position.get(), 1});
        }

        auto set_desired_velocity(OutputVelocity const velocity) -> void {
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
                    this->m_error_state = err2str(static_cast<bmc_error_t>(decoded.fault_code));
                } else if constexpr (std::is_same_v<T, BMCAck>) {
                    RCLCPP_INFO(m_node->get_logger(), "BMC Ack received: %u", decoded.data);
                }
            },
                       msg);
        }
    };
} // namespace mrover
