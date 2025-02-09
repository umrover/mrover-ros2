#pragma once

#include <cstdint>

#include "hardware.hpp"
#include "units.hpp"

namespace mrover {
    class HBridge {
        Pin m_direction_pin{};
#ifdef DUAL_DIRECTION
        Pin m_direction_pin_1{};
#endif
        TIM_HandleTypeDef* m_timer{};
        std::uint32_t m_channel = TIM_CHANNEL_1;
        Percent m_max_pwm{};
        bool m_is_inverted = false;

    public:
        HBridge() = default;

        // explicit HBridge(TIM_HandleTypeDef* timer, std::uint32_t channel, Pin positive_pin, Pin negative_pin);
        explicit HBridge(TIM_HandleTypeDef* timer, std::uint32_t channel, Pin direction_pin);
#ifdef DUAL_DIRECTION
        explicit HBridge(TIM_HandleTypeDef* timer, std::uint32_t channel, Pin direction_pin_0, Pin direction_pin_1);
#endif

        auto write(Percent output) const -> void;

        auto set_direction_pins(Percent duty_cycle) const -> void;

        auto set_duty_cycle(Percent duty_cycle, Percent max_duty_cycle) const -> void;

        auto change_max_pwm(Percent max_pwm) -> void;

        auto change_inverted(bool inverted) -> void;
    };

} // namespace mrover
