#include "encoders.hpp"

#include <cstdint>
#include <optional>

namespace mrover {
    /* ABSOLUTE ENCODER PROCESS:
    1. Request transmission sent (HAL_I2C_Master_Transmit_DMA)
    2. Transmission callback interrupt
    3. Set up read (HAL_I2C_Master_Receive_DMA)
    4. Reception callback interrupt
    5. Update controller
    */

    AbsoluteEncoderReader::AbsoluteEncoderReader(AS5048B_Bus i2c_bus, TIM_HandleTypeDef* elapsed_timer, Radians offset, Ratio multiplier)
        : m_elapsed_timer(elapsed_timer), m_i2cBus{i2c_bus}, m_offset{offset}, m_multiplier{multiplier} {
    }

    auto AbsoluteEncoderReader::request_raw_angle() -> void {
        m_i2cBus.async_transmit(m_address, REQUEST_ANGLE);
    }

    auto AbsoluteEncoderReader::read_raw_angle_into_buffer() -> void {
        m_i2cBus.async_receive(m_address);
    }

    auto AbsoluteEncoderReader::try_read_buffer() -> std::optional<std::uint16_t> {
        std::optional raw_data_optional = m_i2cBus.get_buffer();
        if (!raw_data_optional) return std::nullopt;

        // See: https://github.com/Violin9906/STM32_AS5048B_HAL/blob/0dfcdd2377f332b6bff7dcd948d85de1571d7977/Src/as5048b.c#L28
        // And also the datasheet: https://ams.com/documents/20143/36005/AS5048_DS000298_4-00.pdf
        std::uint16_t raw_data = (raw_data_optional.value()[1] << 6) | (raw_data_optional.value()[0] & 0x3F);
        m_previous_raw_data = raw_data;
        return raw_data;
    }

    /**
     * \return Wrapped angle in the range [-𝜏/2, 𝜏/2), same as [-π, π)
     */
    auto wrap_angle(Radians angle) -> Radians {
        constexpr Radians PI_F{std::numbers::pi};
        return fmod(angle + PI_F, TAU_F) - PI_F;
    }

    [[nodiscard]] auto AbsoluteEncoderReader::read() -> std::optional<EncoderReading> {
    	Seconds elapsed_time;
        if (std::optional<std::uint16_t> counts = try_read_buffer()) {
            std::uint32_t const timer_tick_current = __HAL_TIM_GET_COUNTER(m_elapsed_timer);
            std::uint32_t const tick_diff = timer_tick_current - m_timer_tick_prev;
            Seconds elapsed_time = (1.0f / CLOCK_FREQ) * static_cast<float>(tick_diff);
            m_timer_tick_prev = timer_tick_current;

            // Absolute encoder returns [0, COUNTS_PER_REVOLUTION)
            // We need to convert this to [-𝜏/2, 𝜏/2)
            // Angles always need to be wrapped after addition/subtraction
            m_position = wrap_angle(m_multiplier * Ticks{counts.value()} / ABSOLUTE_CPR + m_offset);
            m_velocity_filter.add_reading(wrap_angle(m_position - m_position_prev) / elapsed_time);
            m_position_prev = m_position;
        }

        return std::make_optional<EncoderReading>(m_position, m_velocity_filter.get_filtered());
    }

} // namespace mrover
