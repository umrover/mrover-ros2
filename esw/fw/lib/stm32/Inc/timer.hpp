#pragma once

#include <array>
#include <cassert>
#include <chrono>
#include <limits>
#include <utility>

#ifdef STM32
#include "main.h"
#endif // STM32

namespace mrover {

#ifdef HAL_TIM_MODULE_ENABLED
    class Timer {
    protected:
        TIM_HandleTypeDef* htim;
        uint32_t sys_clock;
        bool interrupt;
        bool is_en = false;

        auto start_no_it() const -> HAL_StatusTypeDef {
            return HAL_TIM_Base_Start(htim);
        }

        auto start_it() const -> HAL_StatusTypeDef {
            return HAL_TIM_Base_Start_IT(htim);
        }

        auto stop_no_it() const -> HAL_StatusTypeDef {
            return HAL_TIM_Base_Stop(htim);
        }

        auto stop_it() const -> HAL_StatusTypeDef {
            return HAL_TIM_Base_Stop_IT(htim);
        }

    public:
        Timer() = default;
        explicit Timer(TIM_HandleTypeDef* tim_handle, bool const interrupt = false)
            : htim{tim_handle}, sys_clock{HAL_RCC_GetSysClockFreq()}, interrupt{interrupt} {
            start();
        }

        auto start() -> void {
            if (interrupt) {
                check(start_it() == HAL_OK, Error_Handler);
                is_en = true;
            } else {
                check(start_no_it() == HAL_OK, Error_Handler);
                is_en = true;
            }
        }

        auto stop() -> void {
            if (interrupt) {
                check(stop_it() == HAL_OK, Error_Handler);
                is_en = false;
            } else {
                check(stop_no_it() == HAL_OK, Error_Handler);
                is_en = false;
            }
        }

        auto is_enabled() const -> bool {
            return is_en;
        }

        auto get_update_frequency() const -> float {
            uint32_t const psc = htim->Instance->PSC;
            uint32_t const arr = htim->Instance->ARR;

            return static_cast<float>(sys_clock) / ((psc + 1) * (arr + 1));
        }

        auto get_counter_frequency() const -> float {
            return static_cast<float>(sys_clock) / (htim->Instance->PSC + 1);
        }

        auto reset() const -> void {
            __HAL_TIM_SetCounter(htim, 0);
        }
    };
#else  // HAL_TIM_MODULE_ENABLED
    class __attribute__((unavailable("enable 'TIM' in STM32CubeMX to use mrover::Timer"))) Timer {
    public:
        template<typename... Args>
        explicit Timer(Args&&... args) {}
    };
#endif // HAL_TIM_MODULE_ENABLED

    struct TimerConfig {
        std::uint16_t psc;
        std::uint16_t arr;
    };

    constexpr auto configure_timer_16bit(float const tim_frequency, std::chrono::nanoseconds const period) -> TimerConfig {
        uint32_t const expiration_ticks = (tim_frequency * period.count()) / std::nano::den;

        for (uint16_t psc = 1; psc <= 65535; ++psc) {
            if (uint32_t const arr = expiration_ticks / psc; arr > 0 && (arr - 1) <= 65535) {
                return {static_cast<uint16_t>(psc - 1),
                        static_cast<uint16_t>(arr - 1)};
            }
        }

        return {std::numeric_limits<uint16_t>::max(), std::numeric_limits<uint16_t>::max()};
    }

    class IStopwatch {
    public:
        virtual ~IStopwatch() = default;
        typedef void (*TimerCallback)();

        virtual auto remove_stopwatch() -> std::uint8_t = 0;
        virtual auto add_stopwatch() -> std::uint8_t = 0;
        virtual auto add_stopwatch(std::optional<TimerCallback> callback) -> std::uint8_t = 0;
        virtual auto get_time_since_last_read(std::uint8_t index) -> float = 0;
    };

#ifdef HAL_TIM_MODULE_ENABLED
    class ITimerChannel {
    public:
        ITimerChannel() = default;
        virtual ~ITimerChannel() = default;

        virtual auto get_dt() -> float = 0;
        virtual auto forget_reads() -> void = 0;
    };

    template<size_t NumChannels>
    class ElapsedTimer : public Timer {
        class ChannelHandle_t : public ITimerChannel {
            ElapsedTimer* m_parent = nullptr;
            size_t m_channel = 0;

        public:
            ChannelHandle_t() = default;
            ChannelHandle_t(ElapsedTimer& parent, size_t const channel)
                : m_parent(&parent), m_channel(channel) {}

            auto get_dt() -> float override {
                if (!m_parent) return 0.0f;
                assert(m_parent->htim != nullptr && "Dangling Timer Handle Detected!");
                return m_parent->get_time_since_last_read(m_channel);
            }

            auto forget_reads() -> void override {
                if (!m_parent || !m_parent->htim) return;
                m_parent->make_next_read_first_read(m_channel);
            }
        };

        std::array<uint32_t, NumChannels> m_tick_prev{};
        std::array<bool, NumChannels> m_is_first_read{};
        std::array<ChannelHandle_t, NumChannels> m_channels{};

    public:
        ElapsedTimer() = default;
        explicit ElapsedTimer(TIM_HandleTypeDef* tim_handle, bool const interrupt = false)
            : Timer{tim_handle, interrupt} {
            m_is_first_read.fill(true);
            m_tick_prev.fill(0);
            for (size_t channel_id = 0; channel_id < NumChannels; ++channel_id) {
                m_channels[channel_id] = ChannelHandle_t{*this, channel_id};
            }
        }

        ElapsedTimer(ElapsedTimer const&) = delete;
        ElapsedTimer& operator=(ElapsedTimer const&) = delete;
        ElapsedTimer(ElapsedTimer&&) = delete;

        auto get_time_since_last_read(size_t const channel) -> float {
            if (channel >= NumChannels) return 0.0f;

            uint32_t const current_tick = __HAL_TIM_GET_COUNTER(htim);
            float result{0.0f};

            if (m_is_first_read[channel]) {
                m_is_first_read[channel] = false;
            } else {
                uint32_t const arr = htim->Instance->ARR;
                uint32_t const delta_ticks = (current_tick >= m_tick_prev[channel])
                                                     ? (current_tick - m_tick_prev[channel])
                                                     : (arr + 1 - m_tick_prev[channel] + current_tick);

                float const freq = get_counter_frequency();
                result = static_cast<float>(delta_ticks) / freq;
            }

            m_tick_prev[channel] = current_tick;
            return result;
        }

        auto make_next_read_first_read(size_t const channel) -> void {
            if (channel < NumChannels) {
                m_is_first_read[channel] = true;
            }
        }

        auto get_handle(size_t const channel) -> ITimerChannel* {
            return &m_channels[channel];
        }
    };
#else  // HAL_TIM_MODULE_ENABLED
    class __attribute__((unavailable("enable 'TIM' in STM32CubeMX to use mrover::ITimerChannel"))) ITimerChannel {
    public:
        template<typename... Args>
        explicit ITimerChannel(Args&&... args) {}
    };
    class __attribute__((unavailable("enable 'TIM' in STM32CubeMX to use mrover::ElapsedTimer"))) ElapsedTimer {
    public:
        template<typename... Args>
        explicit ElapsedTimer(Args&&... args) {}
    };
#endif // HAL_TIM_MODULE_ENABLED


#ifdef HAL_TIM_MODULE_ENABLED
    template<std::uint8_t MaxStopwatchCount, typename CountType, float TimFrequency>
    class VirtualStopwatches final : public IStopwatch {
        static_assert(std::is_unsigned_v<CountType>, "Template parameter CountType must be an unsigned integer type");
        static_assert(sizeof(CountType) <= 4, "Template parameter CountType must be less than or equal to 32 bits");

    public:
        struct tim_stamp_t {
            CountType last_count{};
            std::size_t num_elapses{};
        };

        struct stopwatch_data_t {
            tim_stamp_t tim_stamp{};
            std::optional<TimerCallback> elapsed_callback{};
        };

        VirtualStopwatches() = default;

        explicit VirtualStopwatches(TIM_HandleTypeDef* htim) : m_hardware_tim(htim) {}

        [[nodiscard]] auto size() const -> std::uint8_t {
            return m_num_stopwatches;
        }

        [[nodiscard]] static auto capacity() -> std::uint8_t {
            return MaxStopwatchCount;
        }

        [[nodiscard]] auto at_capacity() const -> bool {
            return m_num_stopwatches == MaxStopwatchCount;
        }

        auto remove_stopwatch() -> std::uint8_t override {
            return 0;
        }

        auto add_stopwatch() -> std::uint8_t override {
            return add_stopwatch(std::nullopt);
        }

        auto add_stopwatch(std::optional<TimerCallback> callback) -> std::uint8_t override {
            if (m_num_stopwatches >= MaxStopwatchCount) {
                return 0;
            }
            m_stopwatches[m_num_stopwatches].tim_stamp.last_count = get_current_count();
            m_stopwatches[m_num_stopwatches].elapsed_callback = callback;

            return m_num_stopwatches++;
        }

        auto init() const -> void {
            HAL_TIM_Base_Start_IT(m_hardware_tim);
        }

        auto period_elapsed() -> void {
            for (std::size_t i = 0; i < m_num_stopwatches; ++i) {
                ++m_stopwatches[i].tim_stamp.num_elapses;
                if (m_stopwatches[i].elapsed_callback.has_value()) {
                    m_stopwatches[i].elapsed_callback.value()();
                }
            }
        }

        auto get_time_since_last_read(std::uint8_t index) -> float override {
            if (index >= m_num_stopwatches) {
                return static_cast<float>(0.0);
            }

            HAL_TIM_Base_Stop_IT(m_hardware_tim);
            tim_stamp_t last_tim_stamp = std::exchange(m_stopwatches[index].tim_stamp, {get_current_count(), 0});
            HAL_TIM_Base_Start_IT(m_hardware_tim);


            CountType current_count = m_stopwatches[index].tim_stamp.last_count;

            CountType last_count = last_tim_stamp.last_count;
            std::size_t const num_elapses = last_tim_stamp.num_elapses;

            uint64_t elapsed_counts = 0;

            if (current_count >= last_count) {
                elapsed_counts = static_cast<uint64_t>(current_count) - static_cast<uint64_t>(last_count);
            } else {
                elapsed_counts = (static_cast<uint64_t>(std::numeric_limits<CountType>::max()) - static_cast<uint64_t>(last_count) + 1) + static_cast<uint64_t>(current_count);
            }

            if (num_elapses > 0) {
                uint64_t const counts_per_overflow = static_cast<uint64_t>(std::numeric_limits<CountType>::max()) + 1;
                elapsed_counts += num_elapses * counts_per_overflow;
            }

            return 1 / TimFrequency * elapsed_counts;
        }

    private:
        TIM_HandleTypeDef* m_hardware_tim{};
        std::uint8_t m_num_stopwatches{};
        std::array<stopwatch_data_t, MaxStopwatchCount> m_stopwatches{};

        [[nodiscard]] auto get_current_count() const -> CountType {
            return __HAL_TIM_GetCounter(m_hardware_tim);
        }
    };
#else  // HAL_TIM_MODULE_ENABLED
    class __attribute__((unavailable("enable 'TIM' in STM32CubeMX to use mrover::VirtualStopwatches"))) VirtualStopwatches {
    public:
        template<typename... Args>
        explicit VirtualStopwatches(Args&&... args) {}
    };
#endif // HAL_TIM_MODULE_ENABLED

} // namespace mrover
