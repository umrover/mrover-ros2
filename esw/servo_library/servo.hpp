#pragma once


#include <cstdint>
#include <parameter.hpp>
#include <string>
#include <u2d2.hpp>

namespace mrover {

    class Servo {
        using ServoId = uint8_t;
        using ServoPosition = double; // Degrees
        using ServoVelocity = double; // rot/sec
        using ServoCurrent = double;  // mA
        using ServoAddr = uint16_t;

    public:
        enum class ServoProperty {
            PositionPGain = 84,
            PositionIGain = 82,
            PositionDGain = 80,
            VelocityPGain = 78,
            VelocityIGain = 76,
            CurrentLimit = 102,
            ProfileVelocity = 112,
            ProfileAcceleration = 108,
        };

        enum class ServoMode {
            Optimal,
            Clockwise,
            CounterClockwise,
            Limited,
        };

        Servo(rclcpp::Node::SharedPtr node, ServoId id, std::string name);

        auto setPosition(ServoPosition position, ServoMode mode) -> u2d2::Status;
        auto getPosition(ServoPosition& position) const -> u2d2::Status;
        auto getVelocity(ServoVelocity& velocity) const -> u2d2::Status;
        auto getCurrent(ServoCurrent& current) const -> u2d2::Status;
        auto getPositionAbsolute(ServoPosition& position) const -> u2d2::Status;
        auto setProperty(ServoProperty prop, uint16_t value) -> u2d2::Status;
        [[nodiscard]] auto getTargetStatus() const -> u2d2::Status;
        [[nodiscard]] auto getLimitStatus() const -> bool;

    private:
        auto updateConfigFromParameters() -> void;

        ServoId mServoId;
        std::string mServoName;
        int mLimitAdjustment;
        int mAdjustedForwardLimit;
        int mAdjustedReverseLimit;
        uint32_t mGoalPosition;
        float mPositionMultiplier;

        bool mAtLimit = false;

        rclcpp::Node::SharedPtr mNode;
    };

} // namespace mrover