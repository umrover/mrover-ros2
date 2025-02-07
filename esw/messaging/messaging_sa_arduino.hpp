#pragma once

#include <array>
#include <cstdint>
#include <variant>

namespace mrover {

#pragma pack(push, 1)

    static constexpr std::uint8_t HEADER_BYTE = 0xA5;


    struct ServoSetPosition {
        static constexpr std::uint8_t header = HEADER_BYTE;
        static std::uint8_t  message_id = 0x00;

        std::uint8_t _unused: 3{};
        std::uint8_t id : 4 {}; // internal id of servo
        std::uint8_t is_counterclockwise : 1 {};

        float radians{}; 
    };

    struct ServoPositionData {
        static constexpr std::uint8_t header = HEADER_BYTE;
        static std::uint8_t  message_id = 0x01;

        std::uint8_t _unused: 4{};
        std::uint8_t id : 4 {}; // internal id of servo

        float radians{};
    };

    struct TemperatureAndHumidityData {
        static constexpr std::uint8_t header = HEADER_BYTE;
        static std::uint8_t  message_id = 0x02;

        float temperature{};
        float humidity{};
    };
}

// Macros needed to operate on bitfields
#define SET_BIT_AT_INDEX(x, index, value) ((x) = ((x) & ~(1 << (index))) | ((value) << (index)))
#define GET_BIT_AT_INDEX(x, index) ((x) & (1 << (index)))
