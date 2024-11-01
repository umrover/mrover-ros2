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
        TIM_HandleTypeDef* relative_encoder_tick{};
        std::uint8_t absolute_encoder_a2_a1{};
    };

    template<std::uint8_t MotorCount>
    class Controller {
        static_assert(MotorCount > 0 && MotorCount <= 3, "Motor count must be between 1 and 3");

        template<typename Func>
        void foreach_motor(Func func) {
            for (auto& m: m_motors) {
                func(m);
            }
        }

        /* ==================== Hardware ==================== */
        FDCAN<InBoundMessage> m_fdcan;
        VirtualStopwatches<MotorCount * 3, std::uint32_t, mrover::CLOCK_FREQ> m_stopwatches; // MotorCount * 3 = MotorCount(encoder elapsed timer + last throttle timer + last PIDF timer)
        std::array<Motor, MotorCount> m_motors{};
        typename std::array<Motor, MotorCount>::iterator m_motor_requesting_absolute_encoder;

    public:
        Controller() = default;

        Controller(FDCAN<InBoundMessage> const& fdcan, TIM_HandleTypeDef* stopwatch_timer,
                   I2C_HandleTypeDef* absolute_encoder_i2c, std::array<MotorConfig, MotorCount> const& motor_configs)
            : m_fdcan{fdcan},
              m_stopwatches{stopwatch_timer} {
            for (std::size_t i = 0; i < MotorCount; ++i) {
                MotorConfig const& config = motor_configs[i];
                m_motors[i] = Motor{
                        HBridge{config.hbridge_output, config.hbridge_output_channel, config.hbridge_direction},
                        &m_stopwatches,
                        config.receive_watchdog_timer,
                        config.limit_switches,
                        config.relative_encoder_tick,
                        config.absolute_encoder_a2_a1};
            }
            auto it = m_motors.begin();
        }

        auto init() -> void {
            m_stopwatches.init();

            m_fdcan.start();
        }

        /**
         * \brief           Called from the FDCAN interrupt handler when a new message is received, updating \link m_inbound \endlink and processing it.
         * \param message   Command message to process.
         *
         * \note            This resets the message watchdog timer.
         */
        auto receive(FDCAN<InBoundMessage>::MessageId id, InBoundMessage const& message) -> void {
            if (id.destination == DEVICE_ID_0) {
                m_motors[0].receive(message);
                m_motors[0].update();
            } else if (id.destination == DEVICE_ID_1) {
                m_motors[1].receive(message);
                m_motors[1].update();
            } else if (id.destination == DEVICE_ID_2) {
                m_motors[2].receive(message);
                m_motors[2].update();
            }
        }

        /**
         * \brief Send out the current outbound status message of each motor.
         *
         * The update rate should be limited to avoid hammering the FDCAN bus.
         */
        auto send() -> void {
            foreach_motor([this](auto& motor) {
                if (bool success = m_fdcan.broadcast(motor.get_outbound()); !success) {
                    m_fdcan.reset();
                }
            });
        }

        auto request_absolute_encoder_data() -> void {
            foreach_motor([](auto& motor) {
                motor.request_absolute_encoder_data();
            });
        }

        auto read_abolute_encoder_data() -> void {

        }

        template<std::uint8_t MotorIndex>
        auto update_quadrature_encoder() -> void {
            m_motors[MotorIndex].update_quadrature_encoder();
        }

        template<std::uint8_t MotorIndex>
        auto receive_watchdog_expired() -> void {
            m_motors[MotorIndex].receive_watchdog_expired();
        }

        auto virtual_stopwatch_elapsed() -> void {
            m_stopwatches.period_elapsed();
        }
    };
} // namespace mrover
