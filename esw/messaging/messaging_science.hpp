#pragma once

#include <array>
#include <cstdint>
#include <variant>

#include <units.hpp>

namespace mrover {

#pragma pack(push, 1)

    struct BaseScienceMessage {};

    enum struct ScienceDevice {
        HEATER_0,
        HEATER_1,
        WHITE_LED,
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
        std::uint8_t on : 6 {};
    };

    struct HeaterStateData {
        HeaterStateInfo heater_state_info;
    };

    struct ThermistorData {
        std::array<float, 6> temps;
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
