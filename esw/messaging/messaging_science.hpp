#pragma once

#include <array>
#include <cstdint>
#include <variant>

#include <units.hpp>

namespace mrover {

#pragma pack(push, 1)

    struct BaseScienceMessage {};

    enum struct ScienceDevice {
        HEATER_B0,
        HEATER_N0,
        HEATER_B1,
        HEATER_N1,
        HEATER_B2,
        HEATER_N2,
        WHITE_LED_0,
        WHITE_LED_1,
        WHITE_LED_2,
        UV_LED_0,
        UV_LED_1,
        UV_LED_2
    };

    struct EnableScienceDeviceCommand : BaseScienceMessage {
        ScienceDevice science_device{};
        bool enable{};
    };

    struct HeaterAutoShutOffCommand : BaseScienceMessage {
        bool enable_auto_shutoff{};
    };

    struct ConfigThermistorAutoShutOffCommand : BaseScienceMessage {
        float shutoff_temp{};
    };

    struct HeaterStateInfo {
        [[maybe_unused]] std::uint8_t _ignore : 2 {};
        std::uint8_t on : 6 {};
    };

    struct HeaterStateData {
        HeaterStateInfo heater_state_info;
    };

    struct ThermistorData {
        float temps[6];
    };

    struct SensorData {
        std::uint8_t id;
        double data;
    };

    enum class ScienceDataID : uint8_t {
        TEMPERATURE = 1,
        HUMIDITY = 2,
        OXYGEN = 3,
        UV = 4,
    };

    using InBoundScienceMessage = std::variant<
            EnableScienceDeviceCommand, HeaterAutoShutOffCommand, ConfigThermistorAutoShutOffCommand>;

    using OutBoundScienceMessage = std::variant<
            SensorData, HeaterStateData, ThermistorData>;

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