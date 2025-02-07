#pragma once

#include <units.hpp>

namespace mrover {

    constexpr static std::uint8_t MAX_MOTORS = 3;

    constexpr auto TAU_F = 2 * std::numbers::pi_v<float>;

    constexpr auto CLOCK_FREQ = Hertz{170000000};

    // Counts (ticks) per radian (NOT per rotation)
    using CountsPerRad = compound_unit<Ticks, inverse<Radians>>;

    constexpr auto RELATIVE_CPR = CountsPerRad{3355 / TAU_F};      // Measured empirically from the devboard
    constexpr auto ABSOLUTE_CPR = CountsPerRad{(1 << 14) / TAU_F}; // Corresponds to the AS5048B

    // NOTE: Change This For Each Motor Controller
    constexpr static std::uint8_t DEVICE_ID_0 = 0x31; // joint_b
    constexpr static std::uint8_t DEVICE_ID_1 = 0x35; // gripper
    constexpr static std::uint8_t DEVICE_ID_2 = 0x36; // finger

    constexpr static std::uint8_t A2_A1_0 = 0b00;
    constexpr static std::uint8_t A2_A1_1 = 0b11;
    constexpr static std::uint8_t A2_A1_2 = 0b11;

    // Usually this is the Jetson
    constexpr static std::uint8_t DESTINATION_DEVICE_ID = 0x10;

    constexpr static std::uint8_t NUM_MOTORS = 3;

    static_assert(NUM_MOTORS <= MAX_MOTORS, "NUM_MOTORS exceeds the maximum allowed motors");

} // namespace mrover
