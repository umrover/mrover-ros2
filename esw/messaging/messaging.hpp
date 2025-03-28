#pragma once

#include <array>
#include <cstdint>
#include <variant>

#include <units.hpp>

namespace mrover {

#pragma pack(push, 1)

    struct ConfigLimitSwitchInfo {
        std::uint8_t present : 2 {};
        std::uint8_t enabled : 2 {};
        std::uint8_t active_high : 2 {};
        std::uint8_t limits_forward : 2 {}; // limits backward if false
        std::uint8_t use_for_readjustment : 2 {};
        [[maybe_unused]] std::uint8_t _padding_a : 6 {};
        [[maybe_unused]] std::uint8_t _padding_b[2]{};
        std::array<Radians, 2> limit_readj_pos;
    };

    struct ConfigEncoderInfo {
        std::uint8_t quad_present : 1 {};
        std::uint8_t _quad_is_forward_polarity : 1 {};
        std::uint8_t abs_present : 1 {};
        std::uint8_t _abs_is_forward_polarity : 1 {};
        [[maybe_unused]] std::uint8_t _padding_a : 4 {}; // 8 bits - (4 meaningful bits) = 4 ignored bits
        [[maybe_unused]] std::uint8_t _padding_b[3]{};
        Ratio quad_ratio;
        Ratio abs_ratio;
        Radians abs_offset;
    };

    enum struct BDCMCErrorInfo : std::uint8_t {
        NO_ERROR,
        DEFAULT_START_UP_NOT_CONFIGURED,
        RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED,
        RECEIVING_POSITION_COMMANDS_WHEN_NOT_CALIBRATED,
        OUTPUT_SET_TO_ZERO_SINCE_EXCEEDING_LIMITS,
        RECEIVING_PID_COMMANDS_WHEN_NO_READER_EXISTS
    };

    struct ConfigCalibErrorInfo {
        [[maybe_unused]] std::uint8_t _ignore : 2 {}; // 8 bits - (6 meaningful bits) = 2 ignored bits
        std::uint8_t configured : 1 {};
        std::uint8_t calibrated : 1 {};
        BDCMCErrorInfo error : 4 {}; // 0 means no error, anything else is error
    };

    struct LimitStateInfo {
        [[maybe_unused]] std::uint8_t _ignore : 6 {}; // 8 bits - (2 meaningful bits) = 6 ignored bits
        std::uint8_t hit : 2 {};
    };

    struct BaseCommand {
    };

    struct AdjustCommand : BaseCommand {
        Radians position;
    };

    struct ConfigCommand : BaseCommand {
        Dimensionless gear_ratio;
        ConfigLimitSwitchInfo limit_switch_info;
        ConfigEncoderInfo enc_info;
        Percent max_pwm;
        Radians min_position, max_position;
        RadiansPerSecond min_velocity, max_velocity;
        std::uint8_t is_inverted : 1 {};
        [[maybe_unused]] std::uint8_t _ignore : 7 {};
    };

    struct IdleCommand : BaseCommand {
    };

    struct ThrottleCommand : BaseCommand {
        Percent throttle;
    };

    struct VelocityCommand : BaseCommand {
        RadiansPerSecond velocity;
        float p{}, i{}, d{}, ff{};
    };

    struct PositionCommand : BaseCommand {
        Radians position;
        float p{}, i{}, d{};
    };

    struct ControllerDataState {
        Radians position;
        RadiansPerSecond velocity;
        ConfigCalibErrorInfo config_calib_error_data;
        LimitStateInfo limit_switches;
    };

    struct DebugState {
        float f1{};
        float f2{};
        float f3{};
        float f4{};
    };

    using InBoundMessage = std::variant<
            AdjustCommand, ConfigCommand, IdleCommand, ThrottleCommand, VelocityCommand, PositionCommand>;

    using OutBoundMessage = std::variant<
            ControllerDataState, DebugState>;

    struct ArmLaserCommand : BaseCommand {
        bool enable;
    };

    struct LEDInfo {
        [[maybe_unused]] std::uint8_t _ignore : 4 {}; // 8 bits - (4 meaningful bits) = 4 ignored bits
        std::uint8_t red : 1 {};
        std::uint8_t green : 1 {};
        std::uint8_t blue : 1 {};
        std::uint8_t blinking : 1 {};
    };

    struct LEDCommand : BaseCommand {
        LEDInfo led_info;
    };

    struct PDBData : BaseCommand {
        // order is always 24V, then 12v Jetson, 12v rest, 12v buck, 5v, 3v3
        std::array<float, 6> temperatures{};
        std::array<float, 6> currents{};
    };

    using InBoundPDLBMessage = std::variant<
            ArmLaserCommand, LEDCommand>;

    using OutBoundPDLBMessage = std::variant<
            PDBData>;

#pragma pack(pop)

    // Utility for std::visit with lambdas
    template<class... Ts>
    struct overloaded : Ts... {
        using Ts::operator()...;
    };

} // namespace mrover

// Macros needed to operate on bitfields
#define SET_BIT_AT_INDEX(x, index, value) ((x) = ((x) & ~(1 << (index))) | ((value) << (index)))
#define GET_BIT_AT_INDEX(x, index) ((x) & (1 << (index)))
