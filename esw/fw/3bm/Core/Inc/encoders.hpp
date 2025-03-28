#pragma once

#include <cstdint>
#include <optional>

#include <common.hpp>
#include <hardware_i2c.hpp>
#include <hardware_tim.hpp>
#include <units.hpp>

#include "filtering.hpp"

namespace mrover {

    struct EncoderReading {
        Radians position;
        RadiansPerSecond velocity;
    };

    class AbsoluteEncoderReader {
    public:
        using AS5048B_Bus = SMBus<std::uint8_t, std::array<std::uint8_t, 2>>;

        AbsoluteEncoderReader() = default;

        AbsoluteEncoderReader(AS5048B_Bus i2c_bus, uint8_t a2_a1, Radians offset, Ratio multiplier, ElapsedTimer elapsed_timer);

        auto request_raw_angle() -> void;
        auto read_raw_angle_into_buffer() -> void;
        auto try_read_buffer() -> std::optional<std::uint16_t>;

        [[nodiscard]] auto read() -> std::optional<EncoderReading>;

    private:
        constexpr static std::uint16_t REQUEST_ANGLE = 0xFE;

        struct I2CAddress {
            constexpr static std::uint16_t
                    device_slave_address_none_high = 0x40,
                    device_slave_address_a1_high = 0x41,
                    device_slave_address_a2_high = 0x42,
                    device_slave_address_both_high = 0x43;
        };
        ElapsedTimer m_elapsed_timer;

        std::uint16_t m_address = I2CAddress::device_slave_address_none_high;
        AS5048B_Bus m_i2cBus;

        std::uint16_t m_previous_raw_data{};

        Radians m_offset;
        Ratio m_multiplier;

        Radians m_position;
        Radians m_position_prev;
        RunningMeanFilter<RadiansPerSecond, 16> m_velocity_filter;
    };


    class QuadratureEncoderReader {
    public:
        QuadratureEncoderReader() = default;

        QuadratureEncoderReader(TIM_HandleTypeDef* tick_timer, ElapsedTimer elapsed_timer, Ratio multiplier);

        [[nodiscard]] auto read() const -> std::optional<EncoderReading>;

        auto update() -> void;

        auto expired() -> void {
            m_velocity_filter.add_reading(RadiansPerSecond{0});
        }

    private:
        TIM_HandleTypeDef* m_tick_timer{};
        ElapsedTimer m_elapsed_timer;

        std::uint16_t m_counts_unwrapped_prev{};
        Ratio m_multiplier;

        Radians m_position;
        // RadiansPerSecond m_velocity;
        RunningMeanFilter<RadiansPerSecond, 16> m_velocity_filter;
        // IIRFilter<RadiansPerSecond, 0.05> m_velocity_filter;
    };

} // namespace mrover
