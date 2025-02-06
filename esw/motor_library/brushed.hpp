#pragma once

#include <can_device.hpp>
#include <messaging.hpp>
// #include <params_utils.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <units.hpp>

#include "controller.hpp"

namespace mrover {

    struct Gains {
        double p{}, i{}, d{}, ff{};
    };

    // For now only revolute joints are supported => hardcode to Radians
    class BrushedController final : public ControllerBase<Radians, BrushedController> {
    public:
        struct Config {
            static constexpr std::size_t MAX_NUM_LIMIT_SWITCHES = 2;
            std::array<bool, MAX_NUM_LIMIT_SWITCHES> limitSwitchPresent = {false};
            std::array<bool, MAX_NUM_LIMIT_SWITCHES> limitSwitchEnabled = {false};
            std::array<bool, MAX_NUM_LIMIT_SWITCHES> limitSwitchLimitsFwd = {false};
            std::array<bool, MAX_NUM_LIMIT_SWITCHES> limitSwitchActiveHigh = {false};
            std::array<bool, MAX_NUM_LIMIT_SWITCHES> limitSwitchUsedForReadjustment = {false};
            std::array<Radians, MAX_NUM_LIMIT_SWITCHES> limitSwitchReadjustPosition = {0.0_rad};
            bool limitMaxForwardPosition = false;
            bool limitMaxBackwardPosition = false;

            double gearRatio = 0.0;
            bool isInverted = false;

            double driverVoltage = 0.0;
            double motorMaxVoltage = 0.0;

            bool quadPresent = false;
            Ratio quadRatio = 1.0;

            bool absPresent = false;
            Ratio absRatio = 1.0;
            Radians absOffset = 0.0_rad;

            RadiansPerSecond minVelocity = std::numeric_limits<RadiansPerSecond>::infinity();
            RadiansPerSecond maxVelocity = std::numeric_limits<RadiansPerSecond>::infinity();
            Radians minPosition = -std::numeric_limits<Radians>::infinity();
            Radians maxPosition = std::numeric_limits<Radians>::infinity();

            Gains positionGains{};
            Gains velocityGains{};

            Percent calibrationThrottle = 0.0;
        };

        BrushedController(rclcpp::Node::SharedPtr node, std::string masterName, std::string controllerName, Config config);

        auto setDesiredThrottle(Percent throttle) -> void; // from -1.0 to 1.0

        auto setDesiredPosition(Radians position) -> void;

        auto setDesiredVelocity(RadiansPerSecond velocity) -> void;

        auto adjust(Radians position) -> void;

        auto processCANMessage(msg::CAN::ConstSharedPtr const& msg) -> void;

        auto processMessage(ControllerDataState const& state) -> void;

        auto processMessage(DebugState const&) -> void {}

        auto sendConfiguration() -> void;

        auto calibrateServiceCallback(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res) -> void;

    private:
        static auto errorToString(BDCMCErrorInfo errorCode) -> std::string;

        bool mIsConfigured{false};
        ConfigCommand mConfigCommand;

        Gains mPositionGains;
        Gains mVelocityGains;
    };

} // namespace mrover
