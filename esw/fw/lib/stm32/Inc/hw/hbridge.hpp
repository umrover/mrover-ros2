#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <util.hpp>
#include <logger.hpp>

#ifdef STM32
#include "main.h"
#endif // STM32
#include "pin.hpp"

namespace mrover {

#ifdef HAL_TIM_MODULE_ENABLED
    /**
     * \brief Interface to MRover H-Bridge circuit
     */
    class HBridge {
        Pin m_direction_pin{};
        TIM_HandleTypeDef* m_timer{};
        std::uint32_t m_channel{};
        float m_max_pwm{};
        bool m_is_inverted = false;
        bool m_is_pwm_en = false;

        auto set_direction_pins(float const duty_cycle) const -> void {
            GPIO_PinState pin_state;
            if (!m_is_inverted) {
                pin_state = (duty_cycle > 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            } else {
                pin_state = (duty_cycle > 0.0f) ? GPIO_PIN_RESET : GPIO_PIN_SET;
            }
            m_direction_pin.write(pin_state);
        }

        auto set_duty_cycle(float duty_cycle, float const max_duty_cycle) const -> void {
            // Clamp absolute value of the duty cycle to the supported range
            duty_cycle = std::clamp(fabsf(duty_cycle), 0.0f, max_duty_cycle);

            // Set CCR register
            // The CCR register compares its value to the timer and outputs a signal based on the result
            // The ARR register sets the limit for when the timer register resets to 0.
            auto const limit = __HAL_TIM_GetAutoreload(m_timer);
            __HAL_TIM_SetCompare(m_timer, m_channel, static_cast<std::uint32_t>(std::round(duty_cycle * limit)));
            // TODO(eric) we should error if the registers are null pointers
        }

    public:
        HBridge() = default;

        explicit HBridge(TIM_HandleTypeDef* timer, std::uint32_t channel, Pin direction_pin) : m_direction_pin{direction_pin},
                                                                                               m_timer{timer},
                                                                                               m_channel{channel},
                                                                                               m_max_pwm{0.0f} {
            // Initialize CCR to 0 (no pulse generated)
            start();
        }

        auto start() -> void {
            __HAL_TIM_SET_COMPARE(m_timer, m_channel, 0);
            if (!m_is_pwm_en) check(HAL_TIM_PWM_Start(m_timer, m_channel) == HAL_OK, Error_Handler);
            m_is_pwm_en = true;
        }

        auto stop() -> void {
            __HAL_TIM_SET_COMPARE(m_timer, m_channel, 0);
            // check(HAL_TIM_PWM_Stop(m_timer, m_channel) == HAL_OK, Error_Handler);
            // m_is_pwm_en = false;
        }

        auto is_on() const -> bool {
            return m_is_pwm_en;
        }

        auto write(float const output) const -> void {
            // Set direction pins/duty cycle
            set_direction_pins(output);
            set_duty_cycle(output, m_max_pwm);
        }

        auto set_max_pwm(float max_pwm) -> void {
            max_pwm = std::clamp(max_pwm, float{0}, float{1.0});
            m_max_pwm = max_pwm;
        }

        auto set_inverted(bool const inverted) -> void {
            m_is_inverted = inverted;
        }
    };
#else  // HAL_TIM_MODULE_ENABLED
    class __attribute__((unavailable("enable 'TIM' in STM32CubeMX to use mrover::HBridge"))) HBridge {
    public:
        template<typename... Args>
        explicit HBridge(Args&&... args) {}
    };
#endif // HAL_TIM_MODULE_ENABLED

} // namespace mrover
