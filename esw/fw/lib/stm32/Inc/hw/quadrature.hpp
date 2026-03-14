#pragma once

#include <filtering.hpp>
#include <timer.hpp>
#include <util.hpp>

#ifdef STM32
#include "main.h"
#endif // STM32

namespace mrover {

    struct EncoderReading {
        float position; // unit of output
        float velocity; // unit of output/second
    };

#ifdef HAL_TIM_MODULE_ENABLED
    inline auto count_delta_and_update(std::uint16_t& previous, TIM_HandleTypeDef const* timer) -> std::int16_t {
        auto const now = static_cast<std::uint16_t>(__HAL_TIM_GET_COUNTER(timer));
        auto const delta = static_cast<std::int16_t>(now - previous);
        previous = now;
        return delta;
    }

    /**
     * 2-phase quadrature encoder implementation
     *
     * ensure STM32 is configured to read encoder mode on the specified timer
     */
    class QuadratureEncoder {
        static constexpr std::size_t VELOCITY_BUFFER_SIZE = 16;

        TIM_HandleTypeDef* m_tick_timer{};
        ITimerChannel* m_elapsed_timer{};

        std::uint16_t m_counts_unwrapped_prev{};
        float m_multiplier{};
        float m_cpr{};
        bool m_initialized{false};

        float m_position{};
        RunningMeanFilter<float, VELOCITY_BUFFER_SIZE> m_velocity_filter;

    public:
        QuadratureEncoder() = default;
        QuadratureEncoder(TIM_HandleTypeDef* tick_timer, ITimerChannel* elapsed_timer) : m_tick_timer{tick_timer},
                                                                                         m_elapsed_timer{elapsed_timer} {
            check(HAL_TIM_Encoder_Start_IT(m_tick_timer, TIM_CHANNEL_ALL) == HAL_OK, Error_Handler);
        }

        auto init(float const multiplier, float const cpr) -> void {
            m_counts_unwrapped_prev = __HAL_TIM_GET_COUNTER(m_tick_timer);
            m_multiplier = multiplier;
            m_cpr = cpr;
            m_initialized = true;
        }

        [[nodiscard]] auto read() const -> std::optional<EncoderReading> {
            if (!m_initialized) return std::nullopt;
            return std::make_optional(EncoderReading{
                    .position = m_position,
                    .velocity = m_velocity_filter.get_filtered()});
        }

        auto update() -> void {
            if (!m_initialized) return;
            float const elapsed_time = m_elapsed_timer->get_dt();
            std::int16_t const delta_ticks = count_delta_and_update(m_counts_unwrapped_prev, m_tick_timer);
            auto const delta_angle = m_multiplier * static_cast<float>(delta_ticks) / m_cpr;

            m_position += delta_angle;
            m_velocity_filter.add_reading(delta_angle / elapsed_time);
        }

        auto expired() -> void {
            if (!m_initialized) return;
            m_velocity_filter.add_reading(0.0f);
        }
    };
#else  // HAL_TIM_MODULE_ENABLED
    class __attribute__((unavailable("enable 'TIM' in STM32CubeMX to use mrover::QuadratureEncoder"))) QuadratureEncoder {
    public:
        template<typename... Args>
        explicit QuadratureEncoder(Args&&... args) {}
    };
#endif // HAL_TIM_MODULE_ENABLED

} // namespace mrover
