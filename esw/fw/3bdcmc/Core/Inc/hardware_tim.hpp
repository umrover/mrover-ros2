#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <type_traits>
#include <utility>

#include <units/units.hpp>

#include "main.h"

namespace mrover {
    class IStopwatch {
    public:
        using TimerCallback = std::function<void()>;

        virtual auto remove_stopwatch() -> std::uint8_t = 0;
        virtual auto add_stopwatch() -> std::uint8_t = 0;
        virtual auto add_stopwatch(std::optional<TimerCallback> callback) -> std::uint8_t = 0;
        virtual auto get_time_since_last_read(std::uint8_t index) -> Seconds = 0;
    };

    template<std::uint8_t MaxStopwatchCount, typename CountType, Hertz TimFrequency>
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

        auto get_time_since_last_read(std::uint8_t index) -> Seconds override {
            if (index >= m_num_stopwatches) {
                return static_cast<Seconds>(0);
            }

            HAL_TIM_Base_Stop_IT(m_hardware_tim);
            tim_stamp_t last_tim_stamp = std::exchange(m_stopwatches[index].tim_stamp, {get_current_count(), 0});
            HAL_TIM_Base_Start_IT(m_hardware_tim);


            CountType current_count = m_stopwatches[index].tim_stamp.last_count;

            CountType last_count = last_tim_stamp.last_count;
            std::size_t num_elapses = last_tim_stamp.num_elapses;

            uint64_t elapsed_counts = 0;

            if (current_count >= last_count) {
                elapsed_counts = static_cast<uint64_t>(current_count) - static_cast<uint64_t>(last_count);
            } else {
                elapsed_counts = (static_cast<uint64_t>(std::numeric_limits<CountType>::max()) - static_cast<uint64_t>(last_count) + 1) + static_cast<uint64_t>(current_count);
            }

            if (num_elapses > 0) {
                uint64_t counts_per_overflow = static_cast<uint64_t>(std::numeric_limits<CountType>::max()) + 1;
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

} // namespace mrover
