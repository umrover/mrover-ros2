#pragma once

#include <array>
#include <cstdint>
#include <variant>

namespace mrover {

#pragma pack(push, 1)

    struct ServoSetPosition {
        std::uint16_t id : 4 {}; // internal id of servo
        std::uint16_t is_counterclockwise : 1 {};
        std::uint16_t degrees : 9 {};
        std::uint16_t _unused: 2{};
    };

    struct ServoPositionData {
        std::uint16_t id : 4 {}; // internal id of servo
        std::uint16_t degrees : 9 {};
        std::uint16_t _unused: 3{};
    };

    struct TemperatureAndHumidityData {
        float temperature{};
        float humidity{};
    };

}

// Macros needed to operate on bitfields
#define SET_BIT_AT_INDEX(x, index, value) ((x) = ((x) & ~(1 << (index))) | ((value) << (index)))
#define GET_BIT_AT_INDEX(x, index) ((x) & (1 << (index)))
