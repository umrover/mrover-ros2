#pragma once

#include <adc.hpp>

namespace mrover {
#ifdef HAL_ADC_MODULE_ENABLED
    class AD8418A {
    public:
        struct Options {
            float gain{20.0f};
            float shunt_resistance{0.0005f};
            float vref{3.3f};
            float vcm{1.65f};
            uint16_t adc_resolution{4095};
        };

        AD8418A() = default;

        /**
         * @param adc Reference to the ADC wrapper (e.g., ADC<3>)
         * @param channel The index of the channel this sensor is wired to
         */
        AD8418A(ADCBase* adc, uint8_t const channel)
            : m_adc{adc}, m_channel{channel} {}

        auto init(Options const& options, bool const enabled = true) -> void {
            m_enabled = enabled;
            m_options = options;
            m_current = 0.0f;
        }

        auto update_sensor() -> void {
            if (!m_enabled || m_adc == nullptr) return;

            // Retrieve the raw value from the ADC wrapper (DMA or Polling)
            uint32_t const raw_val = 0; //m_adc->get_channel_value(m_channel);

            // Convert to voltage
            float const v_out = (static_cast<float>(raw_val) / static_cast<float>(m_options.adc_resolution)) * m_options.vref;

            /**
             * AD8418A Calculation:
             * Current = (Vout - Vref/2) / (Gain * R_shunt)
             * Note: If your VREF pin on the AD8418A is tied to GND, Vcm should be 0.
             */
            m_current = (v_out - m_options.vcm) / (m_options.gain * m_options.shunt_resistance);
        }

        [[nodiscard]] auto current() const -> float {
            return m_current;
        }

        auto enable() -> void {
            m_enabled = true;
        }

        auto disable() -> void {
            m_enabled = false;
            m_current = 0.0f;
        }

    private:
        ADCBase* m_adc{nullptr};
        uint8_t m_channel{0};

        bool m_enabled{false};
        Options m_options{};

        float m_current{0.0f};
    };
#else  // HAL_ADC_MODULE_ENABLED
    class __attribute__((unavailable("enable 'ADC' in STM32CubeMX to use mrover::AD8418A"))) AD8418A {
    public:
        template<typename... Args>
        explicit AD8418A(Args&&... args) {}
    };
#endif // HAL_ADC_MODULE_ENABLED
} // namespace mrover
