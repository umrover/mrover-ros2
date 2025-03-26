#include "hbridge.hpp"

#include <algorithm>
#include <cmath>

namespace mrover {

    // HBridge::HBridge(TIM_HandleTypeDef* timer, std::uint32_t channel, Pin positive_pin, Pin negative_pin)
    //     : m_positive_pin{positive_pin},
    //       m_negative_pin{negative_pin},

    HBridge::HBridge(TIM_HandleTypeDef* timer, std::uint32_t channel, Pin direction_pin)
        : m_direction_pin{direction_pin},
          m_timer{timer},
          m_channel{channel},
          m_max_pwm{0_percent} {

        // Prevent the motor from spinning on boot up
        __HAL_TIM_SET_COMPARE(m_timer, m_channel, 0);
        check(HAL_TIM_PWM_Start(m_timer, m_channel) == HAL_OK, Error_Handler);
    }

#ifdef DUAL_DIRECTION
    HBridge::HBridge(TIM_HandleTypeDef* timer, std::uint32_t channel, Pin direction_pin_0, Pin direction_pin_1)
        : m_direction_pin{direction_pin_0},
          m_direction_pin_1{direction_pin_1},
          m_timer{timer},
          m_channel{channel},
          m_max_pwm{0_percent} {

        // Prevent the motor from spinning on boot up
        __HAL_TIM_SET_COMPARE(m_timer, m_channel, 0);
        check(HAL_TIM_PWM_Start(m_timer, m_channel) == HAL_OK, Error_Handler);
    }
#endif


    auto HBridge::write(Percent output) const -> void {
        // Set direction pins/duty cycle
        set_direction_pins(output);
        set_duty_cycle(output, m_max_pwm);
    }

    auto HBridge::set_direction_pins(Percent duty_cycle) const -> void {
#ifdef DUAL_DIRECTION
        if (duty_cycle == 0_percent) {
            m_direction_pin.reset();
            m_direction_pin_1.reset();
        } else {
            GPIO_PinState positive_state = duty_cycle > 0_percent ? GPIO_PIN_SET : GPIO_PIN_RESET;
            GPIO_PinState negative_state = duty_cycle < 0_percent ? GPIO_PIN_SET : GPIO_PIN_RESET;
            if (m_is_inverted) std::swap(positive_state, negative_state);
            m_direction_pin.write(positive_state);
            m_direction_pin_1.write(negative_state);
        }
#else
        GPIO_PinState pin_state;
        if (!m_is_inverted) {
            pin_state = (duty_cycle > 0_percent) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        } else {
            pin_state = (duty_cycle > 0_percent) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        }

        m_direction_pin.write(pin_state);
#endif
    }

    auto HBridge::set_duty_cycle(Percent duty_cycle, Percent max_duty_cycle) const -> void {
        // Clamp absolute value of the duty cycle to the supported range
        duty_cycle = std::clamp(abs(duty_cycle), 0_percent, max_duty_cycle);

        // Set CCR register
        // The CCR register compares its value to the timer and outputs a signal based on the result
        // The ARR register sets the limit for when the timer register resets to 0.
        auto limit = __HAL_TIM_GetAutoreload(m_timer);
        __HAL_TIM_SetCompare(m_timer, m_channel, static_cast<std::uint32_t>(std::round(duty_cycle.get() * limit)));
        // TODO(quintin) we should error if the registers are null pointers
    }

    auto HBridge::change_max_pwm(Percent max_pwm) -> void {
        m_max_pwm = max_pwm;
    }

    auto HBridge::change_inverted(bool inverted) -> void {
        m_is_inverted = inverted;
    }

} // namespace mrover
