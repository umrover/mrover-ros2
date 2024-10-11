#pragma once

#include <hardware.hpp>
#include <messaging.hpp>

#include "hbridge.hpp"
#include "encoders.hpp"

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

        /* ==================== Hardware ==================== */
        HBridge m_motor_driver;
        TIM_HandleTypeDef* m_encoder_timer{};
        TIM_HandleTypeDef* m_encoder_elapsed_timer{};
        TIM_HandleTypeDef* m_throttle_timer{};
        TIM_HandleTypeDef* m_pidf_timer{};
        I2C_HandleTypeDef* m_absolute_encoder_i2c{};
        std::optional<QuadratureEncoderReader> m_relative_encoder;
        std::optional<AbsoluteEncoderReader> m_absolute_encoder;
        std::array<LimitSwitch, 2> m_limit_switches;

        /* ==================== Internal State ==================== */
        Mode m_mode;
        BDCMCErrorInfo m_error = BDCMCErrorInfo::DEFAULT_START_UP_NOT_CONFIGURED;
        // "Desired" since it may be overridden.
        // For example if we are trying to drive into a limit switch it will be overridden to zero.
        Percent m_desired_output;
        // Actual output after throttling.
        // Changing the output quickly can result in back-EMF which can damage the board.
        // This is a temporary fix until EHW adds TVS diodes.
        Percent m_throttled_output;
        using PercentPerSecond = compound_unit<Percent, inverse<Seconds>>;
        PercentPerSecond m_throttle_rate{100};
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

    public:
        Motor() = default;

        /**
         * \brief Updates \link m_uncalib_position \endlink and \link m_velocity \endlink based on the hardware
         */
        auto update_relative_encoder() -> void {
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

            Percent output_after_limit = output.value_or(0_percent);
            Percent delta = output_after_limit - m_throttled_output;

            Seconds dt = cycle_time(m_throttle_timer, CLOCK_FREQ);

            Percent applied_delta = m_throttle_rate * dt;

            if (signum(output_after_limit) != signum(m_throttled_output) && signum(m_throttled_output) != 0) {
                // If we are changing directions, go straight to zero
                // This also includes when going to zero from a non-zero value (since signum(0) == 0), helpful for when you want to stop moving quickly on input release
                m_throttled_output = 0_percent;
            } else {
                if (abs(delta) < applied_delta) {
                    // We are very close to the desired output, just set it
                    m_throttled_output = output_after_limit;
                } else {
                    m_throttled_output += applied_delta * signum(delta);
                }
            }

            m_motor_driver.write(m_throttled_output);
        }

    };

} // namespace mrover
