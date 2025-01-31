#include "encoders.hpp"

#include <cstdint>
#include <optional>

namespace mrover {
    /* ABSOLUTE ENCODER PROCESS:
    1. ABSOLUTE_ENCODER_TIMER elapses
    2. Request transmission sent
    3. Transmission callback interrupt
    4. Set up read
    5. Reception callback interrupt
    6. Update controller
    */

    constexpr static std::uint16_t REQUEST_ANGLE = 0xFE;

    AbsoluteEncoderReader::AbsoluteEncoderReader(AS5048B_Bus i2c_bus, uint8_t a2_a1, Radians offset, Ratio multiplier, TIM_HandleTypeDef* elapsed_timer)
        : m_elapsed_timer(elapsed_timer), m_i2cBus{i2c_bus}, m_offset{offset}, m_multiplier{multiplier} {
        // A1/A2 is 1 if pin connected to power, 0 if pin connected to ground
        // This is used for address selection per
        // [datasheet](https://www.mouser.com/datasheet/2/588/AS5048_DS000298_4-00-1100510.pdf?srsltid=AfmBOorG94PYEL0t30_O-gjnl7_jXUCsNwnBYo8pr5MZHPaUmn4QbLmg)
        // I2C Address = 0b10000{a2}{a1}
        uint8_t const a1 = a2_a1 & 0x01;
        uint8_t const a2 = (a2_a1 >> 1) & 0x01;

        if (a1 && a2) {
            m_address = I2CAddress::device_slave_address_both_high;
        } else if (a1) {
            m_address = I2CAddress::device_slave_address_a1_high;
        } else if (a2) {
            m_address = I2CAddress::device_slave_address_a2_high;
        } else {
            m_address = I2CAddress::device_slave_address_none_high;
        }

        check(HAL_TIM_Base_Start_IT(m_elapsed_timer) == HAL_OK, Error_Handler);
    }

    auto AbsoluteEncoderReader::request_raw_angle() -> void {
        m_i2cBus.async_transmit(m_address, REQUEST_ANGLE);
    }

    auto AbsoluteEncoderReader::read_raw_angle_into_buffer() -> void {
        m_i2cBus.async_receive(m_address, m_i2c_buffer);
    }

    auto AbsoluteEncoderReader::convert_buffer_into_raw_angle() -> std::uint64_t {
        // See: https://github.com/Violin9906/STM32_AS5048B_HAL/blob/0dfcdd2377f332b6bff7dcd948d85de1571d7977/Src/as5048b.c#L28
        // And also the datasheet: https://ams.com/documents/20143/36005/AS5048_DS000298_4-00.pdf
        std::uint16_t angle = (m_i2c_buffer[1] << 6) | (m_i2c_buffer[0] & 0x3F);
        m_previous_raw_angle = angle;
        return angle;
    }

    /**
     * \return Wrapped angle in the range [-𝜏/2, 𝜏/2), same as [-π, π)
     */
    auto wrap_angle(Radians angle) -> Radians {
        constexpr Radians PI_F{std::numbers::pi};
        return fmod(angle + PI_F, TAU_F) - PI_F;
    }

    [[nodiscard]] auto AbsoluteEncoderReader::read() -> std::optional<EncoderReading> {
        std::uint64_t const angle = convert_buffer_into_raw_angle();
        Seconds const elapsed_time = cycle_time(m_elapsed_timer, CLOCK_FREQ);

        // Absolute encoder returns [0, COUNTS_PER_REVOLUTION)
        // We need to convert this to [-𝜏/2, 𝜏/2)
        // Angles always need to be wrapped after addition/subtraction
        m_position = wrap_angle(m_multiplier * Ticks{angle} / ABSOLUTE_CPR + m_offset);
        m_velocity_filter.add_reading(wrap_angle(m_position - m_position_prev) / elapsed_time);
        m_position_prev = m_position;

        return std::make_optional<EncoderReading>(m_position, m_velocity_filter.get_filtered());
    }

} // namespace mrover
