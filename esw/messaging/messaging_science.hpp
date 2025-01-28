#pragma once

#include <array>
#include <cstdint>
#include <variant>

namespace mrover {

#pragma pack(push, 1)

    struct BaseCommand {
    };

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

    struct EnableScienceDeviceCommand : BaseCommand {
        ScienceDevice science_device{};
        bool enable{};
    };

    struct HeaterAutoShutOffCommand : BaseCommand {
        bool enable_auto_shutoff{};
    };

    struct ConfigThermistorAutoShutOffCommand : BaseCommand {
        float shutoff_temp{};
    };

    struct HeaterStateInfo {
        [[maybe_unused]] std::uint8_t _ignore : 2 {};
        std::uint8_t on : 6 {};
    };

    struct HeaterStateData : BaseCommand {
        HeaterStateInfo heater_state_info;
    };

    struct ThermistorData : BaseCommand {
        std::array<float, 6> temps{};
    };

    struct SensorData : BaseCommand {
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

} // namespace mrover
