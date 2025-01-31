#pragma once

#include <array>
#include <cstdint>
#include <variant>

namespace mrover {

#pragma pack(push, 1)

    struct sa_arduino_info {
        std::uint8_t id : 3 {};
        std::uint16_t degrees : 9 {};
    };

}

// Macros needed to operate on bitfields
#define SET_BIT_AT_INDEX(x, index, value) ((x) = ((x) & ~(1 << (index))) | ((value) << (index)))
#define GET_BIT_AT_INDEX(x, index) ((x) & (1 << (index)))
