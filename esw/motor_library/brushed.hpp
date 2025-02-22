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
        static constexpr std::size_t MAX_NUM_LIMIT_SWITCHES = 2;
        static_assert(MAX_NUM_LIMIT_SWITCHES <= 2, "Only 2 limit switches are supported");

        Gains mPositionGains{};
        Gains mVelocityGains{};

        Percent mCalibrationThrottle = 0.0;

        bool mIsConfigured{false};
        ConfigCommand mConfigCommand;

    public:
        BrushedController(rclcpp::Node::SharedPtr node, std::string masterName, std::string controllerName);

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

        auto updateConfigFromParameters() -> void;
    };

} // namespace mrover
