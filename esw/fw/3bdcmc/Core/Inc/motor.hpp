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
    };

} // namespace mrover
