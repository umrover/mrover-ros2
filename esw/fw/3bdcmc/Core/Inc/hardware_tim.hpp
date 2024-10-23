#pragma once

#include <array>
#include <limits>
#include <utility>
#include <cstdint>
#include <type_traits>

#include <units/units.hpp>

#include "main.h"

namespace mrover {
class IStopwatch {
    public:
        virtual ~IStopwatch() = default;

        virtual auto add_stopwatch() -> std::uint8_t = 0;
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

        // TODO(owen): terrified of race conditions. what interrupts can happen that screw this up?
        auto add_stopwatch() -> std::uint8_t override {
            if (m_num_stopwatches >= MaxStopwatchCount-1) {
                return 0;
            }
            ++m_num_stopwatches;
            m_tim_stamps[m_num_stopwatches].last_count = get_current_count();

            return m_num_stopwatches;
        }

        auto init() const -> void {
            HAL_TIM_Base_Start_IT(m_hardware_tim);
        }

        auto period_elapsed_callback() -> void {
            for (std::size_t i = 0; i < m_num_stopwatches; ++i) {
                ++m_tim_stamps[i].num_elapses;
            }
        }

        auto get_time_since_last_read(std::uint8_t index) -> Seconds override {
            if (index >= m_num_stopwatches) {
                return static_cast<Seconds>(0);
            }

            HAL_TIM_Base_Stop_IT(m_hardware_tim);
            tim_stamp_t last_tim_stamp = std::exchange(m_tim_stamps[index], {get_current_count(), 0});
            HAL_TIM_Base_Start_IT(m_hardware_tim);


            CountType current_count = m_tim_stamps[index].last_count;

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
        std::array<tim_stamp_t, MaxStopwatchCount> m_tim_stamps{};

        [[nodiscard]] auto get_current_count() const -> CountType {
            return __HAL_TIM_GetCounter(m_hardware_tim);
        }
    };

    }