#include "encoders.hpp"

namespace mrover {


    QuadratureEncoderReader::QuadratureEncoderReader(TIM_HandleTypeDef* tick_timer, ElapsedTimer elapsed_timer, Ratio multiplier)
        : m_tick_timer{tick_timer}, m_elapsed_timer(elapsed_timer), m_multiplier{multiplier} {

        // // Per Sashreek's hypothesis in `Motor velocity calc.pdf` #esw-brushed-24
        // compound_unit<Ticks, inverse<Seconds>> min_measurable_velocity = MIN_MEASURABLE_VELOCITY * RELATIVE_CPR;
        // m_velocity_dt = Seconds{1 / min_measurable_velocity.get()};
        //
        // auto psc = static_cast<std::uint32_t>((CLOCK_FREQ * m_velocity_dt).get() / static_cast<float>(m_timer->Instance->ARR + 1) - 1);
        // m_timer->Instance->PSC = psc;

        m_counts_unwrapped_prev = __HAL_TIM_GET_COUNTER(m_tick_timer);
        check(HAL_TIM_Encoder_Start_IT(m_tick_timer, TIM_CHANNEL_ALL) == HAL_OK, Error_Handler);
    }

    auto count_delta(std::int64_t previous, TIM_HandleTypeDef* timer) -> std::int64_t {
        std::uint32_t const now = __HAL_TIM_GET_COUNTER(timer);
        std::uint32_t const max_value = __HAL_TIM_GET_AUTORELOAD(timer);
        bool const is_counting_down = __HAL_TIM_IS_TIM_COUNTING_DOWN(timer);

        std::int64_t delta;
        if (now == previous) {
            delta = 0;
        } else if (now > previous) {
            if (is_counting_down) {
                delta = -previous - (max_value - now);
            } else {
                delta = now - previous;
            }
        } else /* now < previous */ {
            if (is_counting_down) {
                delta = now - previous;
            } else {
                delta = now + (max_value - previous);
            }
        }
        return delta;
    }

    [[nodiscard]] auto QuadratureEncoderReader::read() const -> std::optional<EncoderReading> {
        return std::make_optional<EncoderReading>(m_position, m_velocity_filter.get_filtered());
        // return std::make_optional<EncoderReading>(m_position, m_velocity);
    }

    auto QuadratureEncoderReader::update() -> void {
        Seconds const elapsed_time = m_elapsed_timer.get_time_since_last_read();
        std::int64_t const delta_ticks = count_delta(m_counts_unwrapped_prev, m_tick_timer);
        m_counts_unwrapped_prev = m_counts_unwrapped_prev + delta_ticks;
        Radians const delta_angle = m_multiplier * Ticks{delta_ticks} / RELATIVE_CPR;

        m_position += delta_angle;
        // m_velocity = delta_angle / elapsed_time;
        m_velocity_filter.add_reading(delta_angle / elapsed_time);
    }

} // namespace mrover
