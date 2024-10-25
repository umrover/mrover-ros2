#pragma once

#include <array>
#include <optional>
#include <variant>

#include <hardware.hpp>
#include <messaging.hpp>
#include <units/units.hpp>

#include "common.hpp"
#include "encoders.hpp"
#include "hbridge.hpp"
#include "motor.hpp"

namespace mrover {
    struct MotorConfig {
        TIM_HandleTypeDef* hbridge_output{};
        std::uint32_t hbridge_output_channel{};
        Pin hbridge_direction{};
        TIM_HandleTypeDef* receive_watchdog_timer{};
        std::array<LimitSwitch, 2> limit_switches{};
        TIM_HandleTypeDef* quad_encoder_tick{};
    };

    template<std::size_t MotorCount>
    class Controller {
        static_assert(MotorCount > 0 && MotorCount <= 3, "Motor count must be between 1 and 3");

        template<typename Func>
        void foreach_motor(Func func) {
            for (auto& m: m_motors) {
                func(m.value());
            }
        }

        /* ==================== Hardware ==================== */
        FDCAN<InBoundMessage> m_fdcan;
        VirtualStopwatches<MotorCount * 3, std::uint32_t, mrover::CLOCK_FREQ> m_stopwatches; // MotorCount * 3 = MotorCount(encoder elapsed timer + last throttle timer + last PIDF timer)
        std::array<Motor, MotorCount> m_motors{};

        /* ==================== Per-Motor Functions ==================== */
        auto update_relative_encoder() -> void {
            foreach_motor([](auto& motor) { motor.update_relative_encoder(); });
        }

        auto update_limit_switches() -> void {
            foreach_motor([](auto& motor) { motor.update_limit_switches(); });
        }

        auto drive_motor() -> void {
            foreach_motor([](auto& motor) { motor.drive_motor(); });
        }


    public:
        Controller() = default;

        Controller(FDCAN<InBoundMessage> const& fdcan, TIM_HandleTypeDef* stopwatch_timer,
                   I2C_HandleTypeDef* absolute_encoder_i2c, std::array<MotorConfig, MotorCount> const& motor_configs)
            : m_fdcan{fdcan},
              m_stopwatches{stopwatch_timer} {
            for (std::size_t i = 0; i < MotorCount; ++i) {
                MotorConfig const& config = motor_configs;
                m_motors[i] = Motor{
                        HBridge{config.hbridge_output, config.hbridge_output_channel, config.hbridge_direction},
                        &m_stopwatches,
                        config.limit_switches};
            }
        }

        auto init() -> void {
            assert_param(!m_stopwatches.at_capacity());
            m_stopwatches.init();

            m_fdcan.start();
        }

        /**
         * \brief Send out the current outbound status message.
         *
         * The update rate should be limited to avoid hammering the FDCAN bus.
         */
        auto send() -> void {
            foreach_motor([](auto& motor) { motor.update(); });
            if (bool success = m_fdcan.broadcast(m_outbound); !success) {
                m_fdcan.reset();
            }
        }

    };
} // namespace mrover
