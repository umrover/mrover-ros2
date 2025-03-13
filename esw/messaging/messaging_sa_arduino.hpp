#pragma once

#include <array>
#include <cstdint>

namespace mrover::messaging {
    namespace arduino {
#pragma pack(push, 1)

        static constexpr std::uint8_t HEADER_BYTE = 0xA5;
        static constexpr std::uint8_t SERVO_SET_POSITION = 0x00;
        static constexpr std::uint8_t SERVO_POSITION_DATA = 0x01;
        static constexpr std::uint8_t TEMPERATURE_HUMIDITY_DATA = 0x02;

        struct ServoSetPosition {
            std::uint8_t header = HEADER_BYTE;
            std::uint8_t messageID = SERVO_SET_POSITION;

            std::uint8_t _unused : 3 {};
            std::uint8_t id : 4 {}; // internal id of servo
            std::uint8_t isCounterClockwise : 1 {};

            float radians{};
        };

        struct ServoPositionData {
            std::uint8_t header = HEADER_BYTE;
            std::uint8_t messageID = SERVO_POSITION_DATA;

            std::uint8_t _unused : 4 {};
            std::uint8_t id : 4 {}; // internal id of servo

            float radians{};
        };

        struct TemperatureAndHumidityData {
            std::uint8_t header = HEADER_BYTE;                  // 1B
            std::uint8_t messageID = TEMPERATURE_HUMIDITY_DATA; // 1B

            float temperature{}; // 4B in degrees Celciues
            float humidity{};    // 4B
        };


#pragma pack(pop)

    } // namespace arduino


} // namespace mrover::messaging

// Macros needed to operate on bitfields
#define SET_BIT_AT_INDEX(x, index, value) ((x) = ((x) & ~(1 << (index))) | ((value) << (index)))
#define GET_BIT_AT_INDEX(x, index) ((x) & (1 << (index)))
