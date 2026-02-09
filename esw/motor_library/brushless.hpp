#pragma once

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <moteus/moteus.h>
#include <moteus/moteus_multiplex.h>

#include <controller.hpp>
#include <parameter.hpp>

namespace mrover {

    using namespace mjbots;

    enum class Mode {
        Stopped = 0,
        Fault = 1,
        PreparingToOperate1 = 2,
        PreparingToOperate2 = 3,
        PreparingToOperate3 = 4,
        PWMMode = 5,
        VoltageMode = 6,
        VoltageFOC = 7,
        VoltageDQ = 8,
        Current = 9,
        Position = 10,
        Timeout = 11,
        ZeroVelocity = 12,
        StayWithin = 13,
        MeasureInductance = 14,
        Brake = 15
    };

    enum class ErrorCode {
        DmaStreamTransferError = 1,
        DmaStreamFifiError = 2,
        UartOverrunError = 3,
        UartFramingError = 4,
        UartNoiseError = 5,
        UartBufferOverrunError = 6,
        UartParityError = 7,
        CalibrationFault = 32,
        MotorDriverFault = 33,
        OverVoltage = 34,
        EncoderFault = 35,
        MotorNotConfigured = 36,
        PwmCycleOverrun = 37,
        OverTemperature = 38,
        StartOutsideLimit = 39,
        UnderVoltage = 40,
        ConfigChanged = 41,
        ThetaInvalid = 42,
        PositionInvalid = 43,
    };

    struct MoteusLimitSwitchInfo {
        bool is_forward_pressed{};
        bool is_backward_pressed{};
    };

    template<IsUnit TOutputPosition>
    class BrushlessController final : public ControllerBase<BrushlessController<TOutputPosition>> {
    public:
        using OutputPosition = TOutputPosition;
        using OutputVelocity = compound_unit<OutputPosition, inverse<Seconds>>;

    private:
        using Base = ControllerBase<BrushlessController>;

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

        enum class MoteusAuxNumber : int {
            AUX1 = 1,
            AUX2 = 2,
        };

        enum class MoteusAuxPin : int {
            PIN0 = 0,
            PIN1 = 1,
            PIN2 = 2,
            PIN3 = 3,
            PIN4 = 4,
        };

        struct LimitSwitchInfo {
            bool present = false;
            bool enabled = true;
            bool limits_forward = false;
            bool active_high = true;
            bool used_for_readjustment = false;
            OutputPosition readjust_position = OutputPosition{0.0};
            MoteusAuxNumber aux_number = MoteusAuxNumber::AUX1;
            MoteusAuxPin aux_pin = MoteusAuxPin::PIN0;
        };

        constexpr static std::size_t MAX_NUM_LIMIT_SWITCHES = 2;
        static_assert(MAX_NUM_LIMIT_SWITCHES <= 2, "Only 2 limit switches are supported");

        OutputVelocity m_min_velocity = OutputVelocity{-1.0};
        OutputVelocity m_max_velocity = OutputVelocity{1.0};
        OutputPosition m_min_position = OutputPosition{-1.0};
        OutputPosition m_max_position = OutputPosition{1.0};
        double m_max_torque = 0.3;
        double m_watchdog_timeout = 0.25;
        double m_current_effort{std::numeric_limits<double>::quiet_NaN()};
        std::array<LimitSwitchInfo, MAX_NUM_LIMIT_SWITCHES> m_limit_switches_info{};

        std::optional<moteus::Controller> m_moteus;
        std::int8_t m_moteus_aux1_info{}, m_moteus_aux2_info{};
        bool m_has_limit{};

    public:
        BrushlessController(rclcpp::Node::SharedPtr node, std::string master_name, std::string controller_name)
            : Base{std::move(node), std::move(master_name), std::move(controller_name)} {

            double min_velocity, max_velocity;
            double min_position, max_position;
            std::vector<ParameterWrapper> parameters = {
                    {std::format("{}.min_velocity", m_controller_name), min_velocity, -1.0},
                    {std::format("{}.max_velocity", m_controller_name), max_velocity, 1.0},
                    {std::format("{}.min_position", m_controller_name), min_position, -1.0},
                    {std::format("{}.max_position", m_controller_name), max_position, 1.0},
                    {std::format("{}.max_torque", m_controller_name), m_max_torque, 0.3},
                    {std::format("{}.watchdog_timeout", m_controller_name), m_watchdog_timeout, 0.25},
            };

            for (std::size_t i = 0; i < MAX_NUM_LIMIT_SWITCHES; ++i) {
                parameters.emplace_back(std::format("{}.limit_switch_{}_present", m_controller_name, i), m_limit_switches_info[i].present, false);
                parameters.emplace_back(std::format("{}.limit_switch_{}_enabled", m_controller_name, i), m_limit_switches_info[i].enabled, true);
                parameters.emplace_back(std::format("{}.limit_switch_{}_limits_forward", m_controller_name, i), m_limit_switches_info[i].limits_forward, false);
                parameters.emplace_back(std::format("{}.limit_switch_{}_active_high", m_controller_name, i), m_limit_switches_info[i].active_high, true);
                parameters.emplace_back(std::format("{}.limit_switch_{}_used_for_readjustment", m_controller_name, i), m_limit_switches_info[i].used_for_readjustment, false);
                parameters.emplace_back(std::format("{}.limit_switch_{}_readjust_position", m_controller_name, i), m_limit_switches_info[i].readjust_position.rep, 0.0);
                parameters.emplace_back(std::format("{}.limit_switch_{}_aux_number", m_controller_name, i), m_limit_switches_info[i].aux_number, MoteusAuxNumber::AUX1);
                parameters.emplace_back(std::format("{}.limit_switch_{}_aux_pin", m_controller_name, i), m_limit_switches_info[i].aux_pin, MoteusAuxPin::PIN0);
            }

            ParameterWrapper::declareParameters(m_node.get(), parameters);

            m_min_velocity = OutputVelocity{min_velocity};
            m_max_velocity = OutputVelocity{max_velocity};
            m_min_position = OutputPosition{min_position};
            m_max_position = OutputPosition{max_position};

            // if active low, we want to make the default value make it believe that
            // the limit switch is NOT pressed.
            // This is because we may not receive the newest query message from the moteus
            // as a result of either testing or startup.
            for (std::size_t i = 0; i < MAX_NUM_LIMIT_SWITCHES; ++i) {
                if (m_limit_switches_info[i].present && m_limit_switches_info[i].enabled) {
                    m_has_limit = true;
                }

                if (m_limit_switches_info[i].present && m_limit_switches_info[i].enabled && !m_limit_switches_info[i].active_high) {
                    if (m_limit_switches_info[i].aux_number == MoteusAuxNumber::AUX1) {
                        m_moteus_aux1_info |= (1 << static_cast<std::size_t>(m_limit_switches_info[i].aux_pin));
                    } else if (m_limit_switches_info[i].aux_number == MoteusAuxNumber::AUX2) {
                        m_moteus_aux2_info |= (1 << static_cast<std::size_t>(m_limit_switches_info[i].aux_pin));
                    }
                }
            }

            moteus::Controller::Options options;
            moteus::Query::Format query_format{};
            query_format.aux1_gpio = moteus::kInt8;
            query_format.aux2_gpio = moteus::kInt8;
            if (this->m_controller_name.find("joint_de") != std::string::npos) {
                // DE0 and DE1 have absolute encoders
                // They are not used for their internal control loops
                // Instead we request them at the ROS level and send adjust commands periodically
                // Therefore we do not get them as part of normal messages and must request them explicitly
                query_format.extra[0] = moteus::Query::ItemFormat{
                        .register_number = moteus::Register::kEncoder1Position,
                        .resolution = moteus::kFloat,
                };
                query_format.extra[1] = moteus::Query::ItemFormat{
                        .register_number = moteus::Register::kEncoder1Velocity,
                        .resolution = moteus::kFloat,
                };
            }
            options.query_format = query_format;
            m_moteus.emplace(options);
        }

        auto set_desired_throttle(Percent throttle) -> void {
#ifdef DEBUG_BUILD
            RCLCPP_DEBUG(m_node->get_logger(), "%s throttle set to: %f. Commanding velocity...", m_controller_name.c_str(), throttle.rep);
#endif
            set_desired_velocity(map_throttle_to_velocity(throttle));
        }

        auto set_desired_velocity(OutputVelocity velocity) -> void {
            // Only check for limit switches if at least one limit switch exists and is enabled
            if (m_has_limit) {
                send_query();

                if (auto [is_fwd_pressed, is_bwd_pressed] = get_pressed_limit_switch_info();
                    (velocity > OutputVelocity{0} && is_fwd_pressed) ||
                    (velocity < OutputVelocity{0} && is_bwd_pressed)) {
#ifdef DEBUG_BUILD
                    RCLCPP_DEBUG(m_node->get_logger(), "%s hit limit switch. Not commanding velocity", m_controller_name.c_str());
#endif
                    set_brake();
                    return;
                }
            }

            velocity = std::clamp(velocity, m_min_velocity, m_max_velocity);

            if (abs(velocity) < OutputVelocity{1e-5}) {
                set_brake();
            } else {
                moteus::PositionMode::Command const command{
                        .position = std::numeric_limits<double>::quiet_NaN(),
                        .velocity = velocity.get(),
                        .maximum_torque = m_max_torque,
                        .watchdog_timeout = m_watchdog_timeout,
                };
                static constexpr moteus::PositionMode::Format format{
                        .position = moteus::kFloat,
                        .velocity = moteus::kFloat,
                        .maximum_torque = moteus::kInt16,
                        .watchdog_timeout = moteus::kFloat,
                };

                moteus::CanFdFrame velocity_frame = m_moteus->MakePosition(command, &format);
                m_device.publish_message(velocity_frame);
            }

#ifdef DEBUG_BUILD
            RCLCPP_DEBUG(m_node->get_logger(), "Commanding %s velocity to: %f", m_controller_name.c_str(), velocity.get());
#endif
        }

        auto set_desired_position(OutputPosition position) -> void {
            // Only check for limit switches if at least one limit switch exists and is enabled
            if (m_has_limit) {
                send_query();

                if (auto [is_fwd_pressed, is_bwd_pressed] = get_pressed_limit_switch_info();
                    (m_position < position.get() && is_fwd_pressed) ||
                    (m_position > position.get() && is_bwd_pressed)) {
#ifdef DEBUG_BUILD
                    RCLCPP_DEBUG(m_node->get_logger(), "%s hit limit switch. Not commanding position", m_controller_name.c_str());
#endif
                    set_brake();
                    return;
                }
            }

            position = std::clamp(position, m_min_position, m_max_position);

            moteus::PositionMode::Command const command{
                    .position = position.get(),
                    .velocity = 0.0,
                    .maximum_torque = m_max_torque,
                    .watchdog_timeout = m_watchdog_timeout,
            };
            static constexpr moteus::PositionMode::Format format{
                    .position = moteus::kFloat,
                    .velocity = moteus::kFloat,
                    .maximum_torque = moteus::kInt16,
                    .watchdog_timeout = moteus::kFloat,
            };

            moteus::CanFdFrame position_frame = m_moteus->MakePosition(command, &format);
            m_device.publish_message(position_frame);

#ifdef DEBUG_BUILD
            RCLCPP_DEBUG(m_node->get_logger(), "Commanding %s position to: %f", m_controller_name.c_str(), position.get());
#endif
        }

        auto process_message(can_msg_t const& msg) -> void {
            std::visit([this](auto const& val) {
                using T = std::decay_t<decltype(val)>;
                if constexpr (std::is_same_v<T, moteus::CanFdFrame>) {
                    auto result = moteus::Query::Parse(val.data, val.size);

                    // Update Base Variables
                    if (this->m_controller_name.find("joint_de") != std::string::npos) {
                        m_position = result.extra[0].value;
                        m_velocity = result.extra[1].value;
                    } else {
                        m_position = static_cast<float>(result.position);
                        m_velocity = static_cast<float>(result.velocity);
                    }
                    m_current = static_cast<float>(result.torque);
                    m_error_state = moteus_error_code_to_error_state(result.mode, static_cast<ErrorCode>(result.fault));
                    m_state = moteus_mode_to_state(result.mode);

                    m_moteus_aux1_info = result.aux1_gpio;
                    m_moteus_aux2_info = result.aux2_gpio;

                    if (result.mode == moteus::Mode::kPositionTimeout || result.mode == moteus::Mode::kFault) {
                        set_stop();
                        RCLCPP_WARN(m_node->get_logger(), "Position timeout hit");
                    }
                }
            },
                       msg);
        }

        auto set_stop() -> void {
            moteus::CanFdFrame set_stop_frame = m_moteus->MakeStop();
            m_device.publish_message(set_stop_frame);
        }

        auto set_brake() -> void {
            moteus::CanFdFrame set_brake_frame = m_moteus->MakeBrake();
            m_device.publish_message(set_brake_frame);
        }

        auto get_pressed_limit_switch_info() -> MoteusLimitSwitchInfo {
            MoteusLimitSwitchInfo result{};
            for (std::size_t i = 0; i < MAX_NUM_LIMIT_SWITCHES; ++i) {
                if (m_limit_switches_info[i].present && m_limit_switches_info[i].enabled) {
                    std::uint8_t const aux_info = (m_limit_switches_info[i].aux_number == MoteusAuxNumber::AUX1) ? m_moteus_aux1_info : m_moteus_aux2_info;
                    bool gpio_state = aux_info & (1 << static_cast<std::size_t>(m_limit_switches_info[i].aux_pin));

                    // Assign to Base m_limit_hit
                    m_limit_hit[i] = (gpio_state == m_limit_switches_info[i].active_high);
                }
                result.is_forward_pressed = (m_limit_hit[i] && m_limit_switches_info[i].limits_forward) || result.is_forward_pressed;
                result.is_backward_pressed = (m_limit_hit[i] && !m_limit_switches_info[i].limits_forward) || result.is_backward_pressed;
                if (m_limit_switches_info[i].used_for_readjustment && m_limit_hit[i]) {
                    adjust(m_limit_switches_info[i].readjust_position);
                }
            }
            return result;
        }

        auto adjust(OutputPosition position) -> void {
            position = std::clamp(position, m_min_position, m_max_position);
            moteus::OutputExact::Command const command{
                    .position = position.get(),
            };
            moteus::OutputExact::Command const outputExactCmd{command};
            moteus::CanFdFrame set_position_frame = m_moteus->MakeOutputExact(outputExactCmd);
            m_device.publish_message(set_position_frame);
        }

        auto send_query() -> void {
            moteus::CanFdFrame queryFrame = m_moteus->MakeQuery();
            m_device.publish_message(queryFrame, true);
        }

    private:
        [[nodiscard]] auto map_throttle_to_velocity(Percent throttle) const -> OutputVelocity {
            throttle = std::clamp(throttle, -1_percent, 1_percent);
            return abs(throttle) * (throttle > 0_percent ? m_max_velocity : m_min_velocity);
        }

        // Converts moteus error codes and mode codes to std::string descriptions
        static auto moteus_error_code_to_error_state(moteus::Mode const motor_mode, ErrorCode const motor_error_code) -> std::string {
            if (motor_mode != moteus::Mode::kFault) return "No Error";
            switch (motor_error_code) {
                case ErrorCode::DmaStreamTransferError:
                    return "DMA Stream Transfer Error";
                case ErrorCode::DmaStreamFifiError:
                    return "DMA Stream FIFO Error";
                case ErrorCode::UartOverrunError:
                    return "UART Overrun Error";
                case ErrorCode::UartFramingError:
                    return "UART Framing Error";
                case ErrorCode::UartNoiseError:
                    return "UART Noise Error";
                case ErrorCode::UartBufferOverrunError:
                    return "UART Buffer Overrun Error";
                case ErrorCode::UartParityError:
                    return "UART Parity Error";
                case ErrorCode::CalibrationFault:
                    return "Calibration Fault";
                case ErrorCode::MotorDriverFault:
                    return "Motor Driver Fault";
                case ErrorCode::OverVoltage:
                    return "Over Voltage";
                case ErrorCode::EncoderFault:
                    return "Encoder Fault";
                case ErrorCode::MotorNotConfigured:
                    return "Motor Not Configured";
                case ErrorCode::PwmCycleOverrun:
                    return "PWM Cycle Overrun";
                case ErrorCode::OverTemperature:
                    return "Over Temperature";
                case ErrorCode::StartOutsideLimit:
                    return "Start Outside Limit";
                case ErrorCode::UnderVoltage:
                    return "Under Voltage";
                case ErrorCode::ConfigChanged:
                    return "Configuration Changed";
                case ErrorCode::ThetaInvalid:
                    return "Theta Invalid";
                case ErrorCode::PositionInvalid:
                    return "Position Invalid";
                default:
                    return "Unknown Error";
            }
        }

        static auto moteus_mode_to_state(moteus::Mode const motor_mode) -> std::string {
            switch (motor_mode) {
                case moteus::Mode::kStopped:
                    return "Motor Stopped";
                case moteus::Mode::kFault:
                    return "Motor Fault";
                case moteus::Mode::kEnabling:
                    return "Motor Enabling";
                case moteus::Mode::kCalibrating:
                    return "Motor Calibrating";
                case moteus::Mode::kCalibrationComplete:
                    return "Motor Calibration Complete";
                case moteus::Mode::kPwm:
                    return "Motor Pwm";
                case moteus::Mode::kVoltage:
                    return "Voltage Operating Mode";
                case moteus::Mode::kVoltageFoc:
                    return "Voltage FOC Operating Mode";
                case moteus::Mode::kVoltageDq:
                    return "Voltage DQ Operating Mode";
                case moteus::Mode::kCurrent:
                    return "Current Operating Mode";
                case moteus::Mode::kPosition:
                    return "Position Operating Mode";
                case moteus::Mode::kPositionTimeout:
                    return "Position Timeout";
                case moteus::Mode::kZeroVelocity:
                    return "Zero Velocity";
                case moteus::Mode::kStayWithin:
                    return "Motor Stay Within";
                case moteus::Mode::kMeasureInd:
                    return "Measure Ind";
                case moteus::Mode::kBrake:
                    return "Motor Brake";
                default:
                    return "Unknown Mode";
            }
        }
    };

} // namespace mrover
