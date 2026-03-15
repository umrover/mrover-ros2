#pragma once


#include <cstdint>
#include <parameter.hpp>
#include <string>
#include <u2d2.hpp>

static constexpr uint16_t DEFAULT_CURRENT_LIMIT = 1750;
static constexpr uint16_t DEFAULT_POSITION_P_GAIN = 400;
static constexpr uint16_t DEFAULT_POSITION_I_GAIN = 0;
static constexpr uint16_t DEFAULT_POSITION_D_GAIN = 0;
static constexpr uint16_t DEFAULT_VELOCITY_P_GAIN = 180;
static constexpr uint16_t DEFAULT_VELOCITY_I_GAIN = 0;

namespace mrover {

    class Servo {
        using ServoId = uint8_t;
        using ServoPosition = float; // Degrees
        using ServoVelocity = float; // rot/sec
        using ServoCurrent = float;  // mA
        using ServoAddr = uint16_t;

    public:
        

        enum class ServoProperty {
            PositionPGain = 84,
            PositionIGain = 82,
            PositionDGain = 80,
            VelocityPGain = 78,
            VelocityIGain = 76,
            CurrentLimit = 102,
        };

        enum class ServoMode {
            Optimal,
            Clockwise,
            CounterClockwise,
            Limited,
        };

        Servo(rclcpp::Node::SharedPtr node, ServoId id, std::string name);

        auto setPosition(ServoPosition position, ServoMode mode) -> u2d2::U2D2Status;
        auto getPosition(ServoPosition& position) -> u2d2::U2D2Status;
        auto getVelocity(ServoVelocity& velocity) -> u2d2::U2D2Status;
        auto getCurrent(ServoCurrent& current) -> u2d2::U2D2Status;
        auto getPositionAbsolute(ServoPosition& position) -> u2d2::U2D2Status;
        auto setProperty(ServoProperty prop, uint16_t value) -> u2d2::U2D2Status;
        auto getTargetStatus() -> u2d2::U2D2Status;
        [[nodiscard]] auto getLimitStatus() const -> bool;

    private:
        auto updateConfigFromParameters() -> void;

        ServoId mServoId;
        std::string mServoName;
        int mForwardLimit;
        int mReverseLimit;
        uint32_t mGoalPosition;
        uint32_t mPositionMultiplier;

        bool mAtLimit = false;

        rclcpp::Node::SharedPtr mNode;
    };

} // namespace mrover