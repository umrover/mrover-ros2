#pragma once

#include <array>
#include <cstdint>

#ifdef STM32
#include "main.h"
#endif // STM32

namespace mrover {

#ifdef HAL_ADC_MODULE_ENABLED
    /**
     * ADCBase class to enable HAL callbacks to be unaware of the number of channels.
     */
    class ADCBase {
    public:
        struct Options {
            uint32_t timeout_ms{10};
            bool use_dma{false};
        };

        virtual uint32_t get_channel_value(size_t index) = 0;
        virtual void handle_conv_complete() = 0;
        virtual ADC_HandleTypeDef* handle() const = 0;
        virtual ~ADCBase() = default;

        static inline ADCBase* s_instance = nullptr;
    };

    /**
     * Analog-to-Digital Converter implementation.
     * @tparam NumChannels The number of ADC channels in the scan sequence
     */
    template<size_t NumChannels>
    class ADC : public ADCBase {
    public:
        ADC() = default;
        explicit ADC(ADC_HandleTypeDef* hadc, Options const& options = Options())
            : m_hadc{hadc}, m_options{options} {
            m_values.fill(0);
            __disable_irq();
            s_instance = this;
            __enable_irq();
            start();
        }

        ADC(const ADC&) = delete;
        ADC& operator=(const ADC&) = delete;
        ADC(ADC&&) noexcept = default;
        ADC& operator=(ADC&&) noexcept = default;

        auto start() -> void {
            __disable_irq();
            if (m_options.use_dma) {
                HAL_ADC_Start_DMA(m_hadc, m_values.data(), static_cast<uint32_t>(NumChannels));
            } else {
                HAL_ADC_Start(m_hadc);
            }
            __enable_irq();
        }

        auto stop() const -> void {
            __disable_irq();
            if (m_options.use_dma) {
                HAL_ADC_Stop_DMA(m_hadc);
            } else {
                HAL_ADC_Stop(m_hadc);
            }
            __enable_irq();
        }

        [[nodiscard]] auto is_data_ready() const -> bool {
            return m_data_ready;
        }

        auto clear_data_ready() -> void {
            __disable_irq();
            m_data_ready = false;
            __enable_irq();
        }

        auto get_channel_value(size_t index) -> uint32_t override {
            __disable_irq();
            uint32_t value = 0;
            index--;
            if (index >= NumChannels) {
                value = 0;
            } else if (m_options.use_dma) {
                value = m_values[index];
            } else if (HAL_ADC_PollForConversion(m_hadc, m_options.timeout_ms) == HAL_OK) {
                value = HAL_ADC_GetValue(m_hadc);
            }
            __enable_irq();
            return value;
        }

        void handle_conv_complete() override {
            m_data_ready = true;
        }

        auto handle() const -> ADC_HandleTypeDef* override {
            return m_hadc;
        }

    private:
        ADC_HandleTypeDef* m_hadc;
        Options m_options;
        std::array<uint32_t, NumChannels> m_values;
        bool volatile m_data_ready{false};
    };
#else  // HAL_ADC_MODULE_ENABLED
    class __attribute__((unavailable("enable 'ADC' in STM32CubeMX to use mrover::ADCBase"))) ADCBase {
    public:
        template<typename... Args>
        explicit ADCBase(Args&&... args) {}
    };

    class __attribute__((unavailable("enable 'ADC' in STM32CubeMX to use mrover::ADC"))) ADC {
    public:
        template<typename... Args>
        explicit ADC(Args&&... args) {}
    };
#endif // HAL_ADC_MODULE_ENABLED

} // namespace mrover
