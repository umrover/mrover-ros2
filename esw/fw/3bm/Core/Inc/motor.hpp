#pragma once

#include <array>
#include <optional>

#include <hardware.hpp>
#include <messaging.hpp>
#include <pidf.hpp>
#include <units.hpp>

#include "encoders.hpp"
#include "hbridge.hpp"

namespace mrover {

    class Motor {
        /* ==================== State Representations ==================== */
        struct PositionMode {
            PIDF<Radians, Percent> pidf;
        };

        struct VelocityMode {
            PIDF<compound_unit<Radians, inverse<Seconds>>, Percent> pidf;
        };

        using Mode = std::variant<std::monostate, PositionMode, VelocityMode>;

        struct StateAfterConfig {
            Dimensionless gear_ratio;
            Radians min_position;
            Radians max_position;
        };

        struct StateAfterCalib {
            // Ex: If the encoder reads in 6 Radians and offset is 4 Radians,
            // Then my actual current position should be 2 Radians.
            Radians offset_position;
        };

        std::uint8_t m_id;
        /* ==================== Hardware ==================== */
        HBridge m_motor_driver;
        TIM_HandleTypeDef* m_receive_watchdog_timer{};
        bool m_receive_watchdog_enabled{};
        std::array<LimitSwitch, 2> m_limit_switches;
        TIM_HandleTypeDef* m_encoder_elapsed_timer{};
        TIM_HandleTypeDef* m_relative_encoder_tick_timer{};
        I2C_HandleTypeDef* m_absolute_encoder_i2c{};
        std::optional<QuadratureEncoderReader> m_relative_encoder;
        std::optional<AbsoluteEncoderReader> m_absolute_encoder;
        std::uint8_t m_absolute_encoder_a2_a1;
        TIM_HandleTypeDef* m_pidf_timer{};

        /* ==================== Internal State ==================== */
        Mode m_mode;
        BDCMCErrorInfo m_error = BDCMCErrorInfo::DEFAULT_START_UP_NOT_CONFIGURED;
        // "Desired" since it may be overridden.
        // For example if we are trying to drive into a limit switch it will be overridden to zero.
        Percent m_desired_output;
        std::optional<Radians> m_uncalib_position;
        std::optional<RadiansPerSecond> m_velocity;
        std::size_t m_missed_absolute_encoder_reads{0};
        // Present if and only if we are configured
        // Configuration messages are sent over the CAN bus
        std::optional<StateAfterConfig> m_state_after_config;
        // Present if and only if we are calibrated
        // Calibrated means we know where we are absolutely
        // This gets set if we hit a limit switch, get an absolute encoder reading, or get a manual adjust command from teleoperation
        std::optional<StateAfterCalib> m_state_after_calib;

        /* ==================== Messaging ==================== */
        InBoundMessage m_inbound = IdleCommand{};
        OutBoundMessage m_outbound = ControllerDataState{.config_calib_error_data = {.error = m_error}};


        /**
         * \brief Updates \link m_uncalib_position \endlink and \link m_velocity \endlink based on the hardware
         */
        auto update_based_on_relative_encoder_reading() -> void {
            if (!m_relative_encoder) return;

            if (std::optional<EncoderReading> reading = m_relative_encoder->read()) {
                auto const& [position, velocity] = reading.value();
                m_uncalib_position = position;
                m_velocity = velocity;
            } else {
                m_uncalib_position.reset();
                m_velocity.reset();
            }
        }

        auto update_limit_switches() -> void {
            // TODO: verify this is correct
            for (LimitSwitch& limit_switch: m_limit_switches) {
                limit_switch.update_limit_switch();
                // Each limit switch may have a position associated with it
                // If we reach there update our offset position since we know exactly where we are

                if (limit_switch.pressed()) {
                    if (std::optional<Radians> readjustment_position = limit_switch.get_readjustment_position()) {
                        if (m_uncalib_position) {
                            if (!m_state_after_calib) m_state_after_calib.emplace();

                            m_state_after_calib->offset_position = m_uncalib_position.value() - readjustment_position.value();
                        }
                    }
                }
            }
        }

        auto drive_motor() -> void {
            std::optional<Percent> output;

            if (m_state_after_config) {
                // TODO: verify this is correct
                bool limit_forward = m_desired_output > 0_percent && (std::ranges::any_of(m_limit_switches, &LimitSwitch::limit_forward)
                                                                      //|| m_uncalib_position > m_state_after_config->max_position
                                                                     );
                bool limit_backward = m_desired_output < 0_percent && (std::ranges::any_of(m_limit_switches, &LimitSwitch::limit_backward)
                                                                       //|| m_uncalib_position < m_state_after_config->min_position
                                                                      );
                if (limit_forward || limit_backward) {
                    m_error = BDCMCErrorInfo::OUTPUT_SET_TO_ZERO_SINCE_EXCEEDING_LIMITS;
                } else {
                    output = m_desired_output;
                }
            }

            m_motor_driver.write(output.value_or(0_percent));
        }

        auto process_command(AdjustCommand const& message) -> void {
            if (m_uncalib_position && m_state_after_config) {
                m_state_after_calib = StateAfterCalib{
                        .offset_position = m_uncalib_position.value() - message.position,
                };
            }
        }

        auto process_command(ConfigCommand const& message) -> void {
            StateAfterConfig config{.gear_ratio = message.gear_ratio};

            if (message.enc_info.quad_present) {
                if (!m_relative_encoder) m_relative_encoder.emplace(m_relative_encoder_tick_timer, m_encoder_elapsed_timer, message.enc_info.quad_ratio);
                EncoderReading enc_read = m_relative_encoder->read().value();
                m_uncalib_position = enc_read.position; // usually but not always 0
            }
            if (message.enc_info.abs_present) {
                if (!m_absolute_encoder) m_absolute_encoder.emplace(AbsoluteEncoderReader::AS5048B_Bus{m_absolute_encoder_i2c}, m_absolute_encoder_a2_a1, message.enc_info.abs_offset, message.enc_info.abs_ratio, m_encoder_elapsed_timer);
            }

            m_motor_driver.change_max_pwm(message.max_pwm);
            m_motor_driver.change_inverted(message.is_inverted);

            config.min_position = message.min_position;
            config.max_position = message.max_position;

            // Boolean configs are stored as bitfields so we have to extract them carefully

            for (std::size_t i = 0; i < m_limit_switches.size(); ++i) {
                if (GET_BIT_AT_INDEX(message.limit_switch_info.present, i)) {
                    const bool enabled = GET_BIT_AT_INDEX(message.limit_switch_info.enabled, i);
                    const bool active_high = GET_BIT_AT_INDEX(message.limit_switch_info.active_high, i);
                    const bool used_for_readjustment = GET_BIT_AT_INDEX(message.limit_switch_info.use_for_readjustment, i);
                    const bool limits_forward = GET_BIT_AT_INDEX(message.limit_switch_info.limits_forward, i);
                    const auto associated_position = Radians{message.limit_switch_info.limit_readj_pos[i]};

                    m_limit_switches[i].initialize(enabled, active_high, used_for_readjustment, limits_forward, associated_position);
                }
            }
            for (std::size_t i = 0; i < m_limit_switches.size(); ++i) {
                if (GET_BIT_AT_INDEX(message.limit_switch_info.present, i) && GET_BIT_AT_INDEX(message.limit_switch_info.enabled, i)) {
                    m_limit_switches[i].enable();
                }
            }

            m_state_after_config = config;
        }

        auto process_command(IdleCommand const&) -> void {
            if (!m_state_after_config) {
                m_error = BDCMCErrorInfo::RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED;
                return;
            }

            m_desired_output = 0_percent;
            m_error = BDCMCErrorInfo::NO_ERROR;
        }

        auto process_command(ThrottleCommand const& message) -> void {
            if (!m_state_after_config) {
                m_error = BDCMCErrorInfo::RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED;
                return;
            }

            m_desired_output = message.throttle;
            m_error = BDCMCErrorInfo::NO_ERROR;
        }

        auto process_command(VelocityCommand const& message, VelocityMode& mode) -> void {
            if (!m_state_after_config) {
                m_error = BDCMCErrorInfo::RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED;
                return;
            }

            if (!m_velocity) {
                m_error = BDCMCErrorInfo::RECEIVING_PID_COMMANDS_WHEN_NO_READER_EXISTS;
                return;
            }

            RadiansPerSecond target = message.velocity;
            RadiansPerSecond input = m_velocity.value();
            mode.pidf.with_p(message.p);
            // mode.pidf.with_i(message.i);
            mode.pidf.with_d(message.d);
            mode.pidf.with_ff(message.ff);
            mode.pidf.with_output_bound(-1.0, 1.0);
            m_desired_output = mode.pidf.calculate(input, target, cycle_time(m_pidf_timer, CLOCK_FREQ));
            m_error = BDCMCErrorInfo::NO_ERROR;

            // m_fdcan.broadcast(OutBoundMessage{DebugState{
            //         .f1 = m_velocity.value().get(),
            //         .f2 = message.velocity.get(),
            // }});
        }

        auto process_command(PositionCommand const& message, PositionMode& mode) -> void {
            if (!m_state_after_config) {
                m_error = BDCMCErrorInfo::RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED;
                return;
            }
            if (!m_state_after_calib) {
                m_error = BDCMCErrorInfo::RECEIVING_POSITION_COMMANDS_WHEN_NOT_CALIBRATED;
                return;
            }

            if (!m_uncalib_position) {
                m_error = BDCMCErrorInfo::RECEIVING_PID_COMMANDS_WHEN_NO_READER_EXISTS;
                return;
            }

            Radians target = message.position;
            Radians input = m_uncalib_position.value() - m_state_after_calib->offset_position;
            mode.pidf.with_p(message.p);
            // mode.pidf.with_i(message.i);
            mode.pidf.with_d(message.d);
            mode.pidf.with_output_bound(-1.0, 1.0);
            m_desired_output = mode.pidf.calculate(input, target, cycle_time(m_pidf_timer, CLOCK_FREQ));
            m_error = BDCMCErrorInfo::NO_ERROR;
        }

        struct detail {
            template<typename Command, typename V>
            struct command_to_mode;

            template<typename Command, typename ModeHead, typename... Modes>
            struct command_to_mode<Command, std::variant<ModeHead, Modes...>> {
                // Linear search to find corresponding mode
                using type = std::conditional_t<requires(Motor motor, Command command, ModeHead mode) { motor.process_command(command, mode); },
                                                ModeHead,
                                                typename command_to_mode<Command, std::variant<Modes...>>::type>; // Recursive call
            };

            template<typename Command> // Base case
            struct command_to_mode<Command, std::variant<>> {
                using type = std::monostate;
            };
        };

        template<typename Command, typename T>
        using command_to_mode_t = typename detail::command_to_mode<Command, T>::type;

    public:
        Motor() = default;

        Motor(std::uint8_t id, HBridge const& motor_driver, TIM_HandleTypeDef* receive_watchdog_timer,
              std::array<LimitSwitch, 2> const& limit_switches, TIM_HandleTypeDef* encoder_elapsed_timer, TIM_HandleTypeDef* relative_encoder_tick_timer,
              std::uint8_t absolute_encoder_a2_a1, TIM_HandleTypeDef* pidf_timer)
            : m_id(id),
              m_motor_driver(motor_driver),
              m_receive_watchdog_timer(receive_watchdog_timer),
              m_limit_switches(limit_switches),
              m_encoder_elapsed_timer(encoder_elapsed_timer),
              m_relative_encoder_tick_timer(relative_encoder_tick_timer),
              m_absolute_encoder_a2_a1(absolute_encoder_a2_a1),
              m_pidf_timer(pidf_timer) {
        }

        [[nodiscard]] auto get_id() const -> std::uint8_t {
            return m_id;
        }

        template<typename Command>
        auto process_command(Command const& command) -> void {
            // Find the "process_command" function that has the right type for the command
            using ModeForCommand = command_to_mode_t<Command, Mode>;

            // If the current mode is not the mode that "process_command" expects, change the mode to a fresh new instance of the correct one
            if (!std::holds_alternative<ModeForCommand>(m_mode)) m_mode.emplace<ModeForCommand>();

            if constexpr (std::is_same_v<ModeForCommand, std::monostate>) {
                process_command(command);
            } else {
                process_command(command, std::get<ModeForCommand>(m_mode));
            }
        }

        // TODO: Abstract the CAN parsing layer to a layer above Motor
        /**
         * \brief           Called from the FDCAN interrupt handler when a new message is received, updating \link m_inbound \endlink and processing it.
         * \param message   Command message to process.
         *
         * \note            This resets the message watchdog timer.
         */
        auto receive(InBoundMessage const& message) -> void {
            // Ensure watchdog timer is reset and enabled now that we are receiving messages
            __HAL_TIM_SetCounter(m_receive_watchdog_timer, 0);
            if (!m_receive_watchdog_enabled) {
                HAL_TIM_Base_Start_IT(m_receive_watchdog_timer);
                m_receive_watchdog_enabled = true;
            }

            m_inbound = message;
        }

        /**
         * \brief Update all non-blocking readings, process the current command stored in \link m_inbound \endlink, update \link m_outbound \endlink, and drive the motor.
         *
         * \note Reading the limit switches and encoders is non-blocking since they are memory-mapped.
         */
        auto process_command() -> void {
            update_limit_switches();
            update_based_on_relative_encoder_reading();
            std::visit([&](auto const& command) { process_command(command); }, m_inbound);
            drive_motor();
        }

        /**
         * \brief Called after not receiving a message for too long. Responsible for stopping the motor for safety.
         *
         * This disables the watchdog timer until we receive another message.
         */
        auto receive_watchdog_expired() -> void {
            HAL_TIM_Base_Stop_IT(m_receive_watchdog_timer);
            m_receive_watchdog_enabled = false;

            // We lost connection or some other error happened
            // Make sure the motor stops
            m_inbound = IdleCommand{};
            process_command();
        }

        /**
         * \brief Get the outbound status message of the controller
         *
         * \return the outbound status message
         */
        [[nodiscard]] auto get_outbound() const -> OutBoundMessage {
            return m_outbound;
        }

        /**
         * \brief Serialize our internal state into an outbound status message
         *
         * \note This does not actually send the message it just updates it. We want to send at a lower rate in \link send() \endlink
         */
        auto update_outbound() -> void {
            ControllerDataState state{
                    // Encoding as NaN instead of an optional saves space in the message
                    // It also has a predictable memory layout
                    .position = [this] {
                        if (m_uncalib_position && m_state_after_calib) {
                            return m_uncalib_position.value() - m_state_after_calib->offset_position;
                        }
                        return Radians{std::numeric_limits<float>::quiet_NaN()};
                    }(),
                    .velocity = m_velocity.value_or(RadiansPerSecond{std::numeric_limits<float>::quiet_NaN()}),
                    .config_calib_error_data = ConfigCalibErrorInfo{
                            .configured = m_state_after_config.has_value(),
                            .calibrated = m_state_after_calib.has_value(),
                            .error = m_error,
                    },
            };
            for (std::size_t i = 0; i < m_limit_switches.size(); ++i) {
                SET_BIT_AT_INDEX(state.limit_switches.hit, i, m_limit_switches[i].pressed());
            }
            m_outbound = state;
        }

        /**
         * \brief Process the last received command again and update our outbound status message.
         *
         * We want to process again since the encoder readings may have changed and we need to update our motor output.
         * We also may have new readings that need to be put in the outbound message.
         *
         * This update loop should be called as fast as possible without overloading the MCU.
         */
        auto update() -> void {
            process_command();
            update_outbound();
        }

        auto encoder_elapsed_expired() -> void {
            if (m_relative_encoder) {
                m_relative_encoder->expired();
                update_quadrature_encoder();
            }
        }

        auto update_quadrature_encoder() -> void {
            if (m_relative_encoder) {
                m_relative_encoder->update();
                update();
            }
        }

        [[nodiscard]] auto has_quadrature_encoder_configured() const -> bool {
            return m_relative_encoder.has_value();
        }

        [[nodiscard]] auto has_absolute_encoder_configured() const -> bool {
            return m_absolute_encoder.has_value();
        }

        // Max hits before we remove the calibration state
        constexpr static std::size_t MAX_MISSED_ABSOLUTE_ENCODER_READS = 32;

        auto request_absolute_encoder_data() -> void {
            // Only read the encoder if we are configured
            if (m_absolute_encoder) {
                m_absolute_encoder->request_raw_angle();
                // TODO(quintin): No magic numbers
                if (m_missed_absolute_encoder_reads++ > MAX_MISSED_ABSOLUTE_ENCODER_READS) {
                    m_state_after_calib.reset();
                }
            }
        }

        auto read_absolute_encoder_data() -> void {
            if (m_absolute_encoder) {
                m_absolute_encoder->read_raw_angle_into_buffer();
            }
        }

        /**
         * Called after a successful I2C transaction
         */
        auto update_absolute_encoder() -> void {
            if (!m_absolute_encoder) return;

            if (std::optional<EncoderReading> reading = m_absolute_encoder->read()) {
                auto const& [position, velocity] = reading.value();
                if (!m_state_after_calib) m_state_after_calib.emplace();

                m_missed_absolute_encoder_reads = 0;

                // TODO(quintin): This is pretty stupid
                m_state_after_calib->offset_position = -position;

                // m_fdcan.broadcast(OutBoundMessage{DebugState{
                //         .f1 = position.get(),
                // }});

                m_uncalib_position.emplace(); // Reset to zero
                m_velocity = velocity;
            } else {
                m_uncalib_position.reset();
                m_velocity.reset();
            }
        }
    };

} // namespace mrover
