#include "encoders.hpp"

namespace mrover {

    QuadratureEncoderReader::QuadratureEncoderReader(TIM_HandleTypeDef* tick_timer, Ratio multiplier, IStopwatch* stopwatch)
        : m_tick_timer{tick_timer}, m_stopwatch(stopwatch), m_multiplier{multiplier} {

        // // Per Sashreek's hypothesis in `Motor velocity calc.pdf` #esw-brushed-24
        // compound_unit<Ticks, inverse<Seconds>> min_measurable_velocity = MIN_MEASURABLE_VELOCITY * RELATIVE_CPR;
        // m_velocity_dt = Seconds{1 / min_measurable_velocity.get()};
        //
        // auto psc = static_cast<std::uint32_t>((CLOCK_FREQ * m_velocity_dt).get() / static_cast<float>(m_timer->Instance->ARR + 1) - 1);
        // m_timer->Instance->PSC = psc;

        m_counts_unwrapped_prev = __HAL_TIM_GetCounter(m_tick_timer);
        check(HAL_TIM_Encoder_Start_IT(m_tick_timer, TIM_CHANNEL_ALL) == HAL_OK, Error_Handler);

        m_stopwatch_id = m_stopwatch->add_stopwatch(std::bind(&QuadratureEncoderReader::expired, this));
    }

    auto count_delta(std::int64_t& store, TIM_HandleTypeDef* timer) -> std::int64_t {
        std::uint32_t now = __HAL_TIM_GetCounter(timer);
        std::uint32_t max_value = __HAL_TIM_GetAutoreload(timer);

        // Adapted from: https://electronics.stackexchange.com/questions/605278/how-to-increase-stm32-timer-encoder-mode-counter-value
        // Handles when the timer wraps around
        std::int64_t c64 = static_cast<std::int64_t>(now) - max_value / 2; // Remove half period to determine (+/-) sign of the wrap
        std::int64_t dif = c64 - store;                                    // prev + (current - prev) = current

        // Wrap difference from -HALF_PERIOD to HALF_PERIOD. The modulo prevents differences after the wrap from having an incorrect result
        std::int64_t mod_dif = (dif + max_value / 2) % max_value - max_value / 2;
        if (dif < -max_value / 2) mod_dif += max_value; // Account for the behavior of the modulo operator with negative numbers in C++

        std::int64_t unwrapped = store + mod_dif;
        store = unwrapped;

        return mod_dif;
    }

    [[nodiscard]] auto QuadratureEncoderReader::read() const -> std::optional<EncoderReading> {
        return std::make_optional<EncoderReading>(m_position, m_velocity_filter.get_filtered());
        // return std::make_optional<EncoderReading>(m_position, m_velocity);
    }

    auto QuadratureEncoderReader::update() -> void {
        Seconds elapsed_time = m_stopwatch->get_time_since_last_read(m_stopwatch_id);
        std::int64_t delta_ticks = count_delta(m_counts_unwrapped_prev, m_tick_timer);
        Radians delta_angle = m_multiplier * Ticks{delta_ticks} / RELATIVE_CPR;

        m_position += delta_angle;
        // m_velocity = delta_angle / elapsed_time;
        m_velocity_filter.add_reading(delta_angle / elapsed_time);
    }

} // namespace mrover
