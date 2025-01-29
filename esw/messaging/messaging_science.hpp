#pragma once

#include <array>
#include <cstdint>
#include <variant>

#include <units.hpp>

namespace mrover {

#pragma pack(push, 1)

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

    struct EnableScienceDeviceCommand {
        ScienceDevice science_device{};
        bool enable{};
    };

    struct HeaterAutoShutOffCommand {
        bool enable_auto_shutoff{};
    };

    struct ConfigThermistorAutoShutOffCommand {
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
        std::array<float, 6> temps{};
    };

    struct SensorData {
        std::uint8_t id;
        double data;
    };

    enum class ScienceDataID : uint8_t {
        TEMPERATURE = 1,
        HUMIDITY = 2,
        OXYGEN = 3,
        METHANE = 4,
        UV = 5,
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