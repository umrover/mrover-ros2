#pragma once

#include <cstdint>

#ifdef STM32
#include "main.h"
#endif // STM32

namespace mrover {

#ifdef HAL_GPIO_MODULE_ENABLED
    class Pin {
        GPIO_TypeDef* m_port{};
        std::uint16_t m_pin{};

    public:
        Pin() = default;

        Pin(GPIO_TypeDef* port, std::uint16_t const pin)
            : m_port{port}, m_pin{pin} {}

        [[nodiscard]] auto read() const -> GPIO_PinState {
            return HAL_GPIO_ReadPin(m_port, m_pin);
        }

        auto write(GPIO_PinState const val) const -> void {
            HAL_GPIO_WritePin(m_port, m_pin, val);
        }

        auto toggle() const -> void {
            HAL_GPIO_TogglePin(m_port, m_pin);
        }

        auto reset() const -> void {
            HAL_GPIO_WritePin(m_port, m_pin, GPIO_PIN_RESET);
        }

        auto set() const -> void {
            HAL_GPIO_WritePin(m_port, m_pin, GPIO_PIN_SET);
        }

        [[nodiscard]] auto is_set() const -> bool {
            return HAL_GPIO_ReadPin(m_port, m_pin) == GPIO_PIN_SET;
        }
    };
#else  // HAL_GPIO_MODULE_ENABLED
    class __attribute__((unavailable("enable 'GPIO' in STM32CubeMX to use mrover::Pin"))) Pin {
    public:
        template<typename... Args>
        explicit Pin(Args&&... args) {}
    };
#endif // HAL_GPIO_MODULE_ENABLED

} // namespace mrover
