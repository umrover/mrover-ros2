#pragma once

#include <units.hpp>

namespace mrover {

    constexpr auto TAU_F = 2 * std::numbers::pi_v<float>;

    constexpr auto CLOCK_FREQ = Hertz{170000000};

    // Counts (ticks) per radian (NOT per rotation)
    using CountsPerRad = compound_unit<Ticks, inverse<Radians>>;

    constexpr auto RELATIVE_CPR = CountsPerRad{8245.92 / TAU_F};      // Measured empirically from the devboard
    constexpr auto ABSOLUTE_CPR = CountsPerRad{(1 << 14) / TAU_F}; // Corresponds to the AS5048B

    // NOTE: Change This For Each Motor Controller
    constexpr static std::uint8_t DEVICE_ID = 0x0A; // currently set for joint_b

    // Usually this is the Jetson
    constexpr static std::uint8_t DESTINATION_DEVICE_ID = 0x05;

    // AS5048B address pins
    constexpr static std::uint8_t A2_A1 = 0b00;
} // namespace mrover
