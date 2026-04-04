#pragma once

#include <parameter.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <utility>

#include "u2d2.hpp"

namespace mrover {

    class Servo {
        using ServoID = uint8_t;
        using ServoPosition = double; // Degrees
        using ServoVelocity = double; // rot/sec
        using ServoCurrent = double;  // mA
        using ServoAddr = uint16_t;

        static constexpr uint8_t ADDR_OPERATING_MODE = 11;
        static constexpr uint8_t ADDR_TORQUE_ENABLE = 64;
        static constexpr uint8_t ADDR_GOAL_POSITION = 116;
        static constexpr uint8_t ADDR_PRESENT_POSITION = 132;
        static constexpr uint8_t ADDR_PRESENT_VELOCITY = 128;
        static constexpr uint8_t ADDR_PRESENT_CURRENT = 126;

        static constexpr int32_t SERVO_TICKS = 4096;
        static constexpr uint8_t SERVO_POSITION_DEAD_ZONE = 5;

        [[nodiscard]] constexpr auto getUpper(int64_t const val) const -> auto { return val == 0 ? static_cast<int64_t>(mTicksPerRevolution) : val; }

        ServoID mServoID;
        std::string mServoName;
        int64_t mLimitAdjustment;
        int64_t mAdjustedForwardLimit;
        int64_t mAdjustedReverseLimit;
        int64_t mGoalPosition;
        uint32_t mPositionOffsetTicks;
        int64_t mTicksPerRevolution;

        bool mAtLimit = false;

        rclcpp::Node::SharedPtr mNode;

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

        Servo(rclcpp::Node::SharedPtr node, std::string servoName) : mServoName{std::move(servoName)}, mLimitAdjustment{0}, mAdjustedForwardLimit{0},
                                                                     mAdjustedReverseLimit{0}, mGoalPosition{0}, mPositionOffsetTicks{0}, mTicksPerRevolution{SERVO_TICKS}, mNode{std::move(node)} {

            int id;
            std::vector<ParameterWrapper> parameters = {
                    {std::format("{}.id", mServoName), id, 0},
            };
            ParameterWrapper::declareParameters(mNode.get(), parameters);
            mServoID = static_cast<ServoID>(id);

            U2D2::getInstance()->registerServo(mServoID);
            updateConfigFromParameters();

            uint8_t hardwareStatus;

            // Use Position Control Mode
            U2D2::getInstance()->write1Byte(ADDR_OPERATING_MODE, 4, mServoID, &hardwareStatus);

            // Enable torque
            U2D2::getInstance()->write1Byte(ADDR_TORQUE_ENABLE, 1, mServoID, &hardwareStatus);
        }


        auto setPosition(ServoPosition const position, ServoMode const mode) -> U2D2::Status {
            // Convert degrees to ticks (0.0 - 360.0) to (0 to mTicksPerRevolution)
            mGoalPosition = static_cast<int64_t>((position / 360.0f) * static_cast<double>(mTicksPerRevolution));

            RCLCPP_INFO_STREAM(mNode->get_logger(), "Setting goal position: " << mGoalPosition);

            auto currentPositionAndStatus = getCurrentServoPosition();

            if (currentPositionAndStatus.second != U2D2::Status::Success) {
                return currentPositionAndStatus.second;
            }

            // Calculate the signed difference (accounting for overflow)
            auto normalizedDifference = static_cast<int64_t>(mGoalPosition - currentPositionAndStatus.first);

            mAtLimit = false;

            switch (mode) {
                case ServoMode::Optimal:
                    if (normalizedDifference > (mTicksPerRevolution / 2)) {
                        mGoalPosition -= mTicksPerRevolution; // Go the other (shorter) way around
                    } else if (normalizedDifference < -(mTicksPerRevolution / 2)) {
                        mGoalPosition += mTicksPerRevolution; // Go the other (shorter) way around
                    }
                    break;
                case ServoMode::Clockwise: // clockwise
                    if (normalizedDifference < 0) {
                        mGoalPosition += mTicksPerRevolution;
                    }
                    break;
                case ServoMode::CounterClockwise: // counter clockwise
                    if (normalizedDifference > 0) {
                        mGoalPosition -= mTicksPerRevolution;
                    }
                    break;
                case ServoMode::Limited: {

                    // Adjust target and current position
                    int64_t const adjustedCurrentPosition = (currentPositionAndStatus.first - mLimitAdjustment + mTicksPerRevolution) % mTicksPerRevolution;
                    int64_t const adjustedTargetPosition = (mGoalPosition - mLimitAdjustment + mTicksPerRevolution) % mTicksPerRevolution;

                    // If the current path to the final position goes over the middle limit, go the other way
                    if (0 > adjustedCurrentPosition && 0 < adjustedTargetPosition) {

                        if (normalizedDifference > 0)
                            mGoalPosition -= mTicksPerRevolution;
                        else if (normalizedDifference < 0)
                            mGoalPosition += mTicksPerRevolution;
                    }

                    mAtLimit = false;

                    // Limit destination if between mForwardLimit and middleLimit
                    if (getUpper(adjustedTargetPosition) > mAdjustedForwardLimit) {
                        mGoalPosition = (mAdjustedForwardLimit - adjustedCurrentPosition) % mTicksPerRevolution;
                        if (normalizedDifference < 0 && !(getUpper(adjustedCurrentPosition) > mAdjustedForwardLimit && adjustedCurrentPosition < mTicksPerRevolution)) mGoalPosition += mTicksPerRevolution;
                        mAtLimit = true;
                    }

                    // Limit destination if between mReverseLimit and middleLimit
                    else if (adjustedTargetPosition < getUpper(mAdjustedReverseLimit)) {

                        mGoalPosition = (mAdjustedReverseLimit - adjustedCurrentPosition) % mTicksPerRevolution;
                        if (normalizedDifference > 0 && !(getUpper(adjustedCurrentPosition) > 0 && adjustedCurrentPosition < getUpper(mAdjustedReverseLimit))) mGoalPosition -= mTicksPerRevolution;
                        mAtLimit = true;
                    }
                }
            }

            // Write goal position
            uint8_t hardwareStatus;
            uint32_t rawGoal = offsetToRaw(mGoalPosition);
            RCLCPP_INFO_STREAM(mNode->get_logger(), "Setting raw position: " << rawGoal);
            return U2D2::getInstance()->write4Byte(ADDR_GOAL_POSITION, rawGoal, mServoID, &hardwareStatus);
        }


        auto getPosition(ServoPosition& position) const -> U2D2::Status {
            auto const positionTicks = getCurrentServoPosition();
            position = (static_cast<double>(positionTicks.first) / static_cast<double>(mTicksPerRevolution)) * 360.0;
            return positionTicks.second;
        }

        auto getVelocity(ServoVelocity& velocity) const -> U2D2::Status {
            uint8_t hardwareStatus;
            uint32_t velocity_int;
            U2D2::Status const status = U2D2::getInstance()->read4Byte(ADDR_PRESENT_VELOCITY, velocity_int, mServoID, &hardwareStatus);
            velocity = (static_cast<double>(velocity_int) * 0.22888); // 0.22888f Conversion factor to get rot/sec (found in dynamixel wizard)
            return status;
        }

        auto getCurrent(ServoCurrent& current) const -> U2D2::Status {
            uint8_t hardwareStatus;
            uint16_t currentInt;
            U2D2::Status const status = U2D2::getInstance()->read2Byte(ADDR_PRESENT_CURRENT, currentInt, mServoID, &hardwareStatus);
            current = static_cast<double>(currentInt) / 1000.0;
            return status;
        }

        [[nodiscard]] auto setProperty(ServoProperty prop, uint16_t const value) const -> U2D2::Status {
            uint8_t hardwareStatus;
            if (prop == ServoProperty::ProfileVelocity || prop == ServoProperty::ProfileAcceleration) {
                return U2D2::getInstance()->write4Byte(static_cast<ServoAddr>(prop), value, mServoID, &hardwareStatus);
            }
            return U2D2::getInstance()->write2Byte(static_cast<ServoAddr>(prop), value, mServoID, &hardwareStatus);
        }

        [[nodiscard]] auto getTargetStatus() const -> U2D2::Status {
            auto const currentPositionAndStatus = getCurrentServoPosition();

            if (currentPositionAndStatus.second != U2D2::Status::Success) return currentPositionAndStatus.second;

            if (std::abs(currentPositionAndStatus.first - mGoalPosition) < SERVO_POSITION_DEAD_ZONE) {
                return U2D2::Status::Success;
            }

            return U2D2::Status::Active;
        }

        [[nodiscard]] auto getLimitStatus() const -> bool {
            return mAtLimit;
        }

    private:
        auto check(bool const condition, std::string const& errMsg) const -> void {
            if (!condition) RCLCPP_ERROR(mNode->get_logger(), "%s", errMsg.c_str());
        }

        auto updateConfigFromParameters() -> void {
            double forwardLimit;
            double reverseLimit;
            double positionPGain;
            double positionIGain;
            double positionDGain;
            double velocityPGain;
            double velocityIGain;
            double currentLimit;
            double profileAcceleration;
            double profileVelocity;
            int ticksPerRevolution;

            std::vector<ParameterWrapper> parameters = {
                    {std::format("{}.ticks_per_revolution", mServoName), ticksPerRevolution, SERVO_TICKS},
                    {std::format("{}.reverse_limit", mServoName), reverseLimit, 0.0},
                    {std::format("{}.forward_limit", mServoName), forwardLimit, 340.0},
                    {std::format("{}.position_p", mServoName), positionPGain, 400.0},
                    {std::format("{}.position_i", mServoName), positionIGain, 0.0},
                    {std::format("{}.position_d", mServoName), positionDGain, 0.0},
                    {std::format("{}.velocity_p", mServoName), velocityPGain, 180.0},
                    {std::format("{}.velocity_i", mServoName), velocityIGain, 90.0},
                    {std::format("{}.current_limit", mServoName), currentLimit, 1750.0},
                    {std::format("{}.profile_acceleration", mServoName), profileAcceleration, 100.0},
                    {std::format("{}.profile_velocity", mServoName), profileVelocity, 100.0}};

            ParameterWrapper::declareParameters(mNode.get(), parameters);

            check(setProperty(ServoProperty::PositionPGain, static_cast<uint16_t>(positionPGain)) == U2D2::Status::Success, "pos p gain error");
            check(setProperty(ServoProperty::PositionIGain, static_cast<uint16_t>(positionIGain)) == U2D2::Status::Success, "pos i gain error");
            check(setProperty(ServoProperty::PositionDGain, static_cast<uint16_t>(positionDGain)) == U2D2::Status::Success, "pos d gain error");
            check(setProperty(ServoProperty::VelocityPGain, static_cast<uint16_t>(velocityPGain)) == U2D2::Status::Success, "vel p gain error");
            check(setProperty(ServoProperty::VelocityIGain, static_cast<uint16_t>(velocityIGain)) == U2D2::Status::Success, "vel i gain error");
            check(setProperty(ServoProperty::CurrentLimit, static_cast<uint16_t>(currentLimit)) == U2D2::Status::Success, "current limit gain error");
            check(setProperty(ServoProperty::ProfileAcceleration, static_cast<uint16_t>(profileAcceleration)) == U2D2::Status::Success, "profile accel gain error");
            check(setProperty(ServoProperty::ProfileVelocity, static_cast<uint16_t>(profileVelocity)) == U2D2::Status::Success, "profile vel gain error");

            ParameterWrapper::declareParameters(mNode.get(), parameters);

            mTicksPerRevolution = ticksPerRevolution;

            int const reverseLimitTicks = static_cast<int>((reverseLimit / 360.0f) * static_cast<double>(mTicksPerRevolution));
            int const forwardLimitTicks = static_cast<int>((forwardLimit / 360.0f) * static_cast<double>(mTicksPerRevolution));

            mLimitAdjustment = (forwardLimitTicks + reverseLimitTicks) / 2;

            if (forwardLimitTicks > reverseLimitTicks) {
                mLimitAdjustment = (forwardLimitTicks + reverseLimitTicks + mTicksPerRevolution) / 2;
            }
            mLimitAdjustment %= mTicksPerRevolution;

            mAdjustedReverseLimit = (static_cast<int64_t>((reverseLimit / 360.0f) * static_cast<double>(mTicksPerRevolution)) - mLimitAdjustment + mTicksPerRevolution) % mTicksPerRevolution;
            mAdjustedForwardLimit = (static_cast<int>((forwardLimit / 360.0f) * static_cast<double>(mTicksPerRevolution)) - mLimitAdjustment + mTicksPerRevolution) % mTicksPerRevolution;

            setOffset();
        }

        auto setOffset() -> void {
            // Read the servos current position
            uint8_t hardwareStatus;
            uint32_t presentPosition;
            U2D2::getInstance()->read4Byte(ADDR_PRESENT_POSITION, presentPosition, mServoID, &hardwareStatus);

            // Update the offset of the servo
            mPositionOffsetTicks = presentPosition;
        }

        [[nodiscard]] auto rawToOffset(uint32_t position) const -> int64_t {
            // apply the offset to the position of the servos
            int64_t updatedPosition = static_cast<int64_t>(position) - mPositionOffsetTicks;

            // correct for an underflow
            while (updatedPosition < 0) {
                updatedPosition += mTicksPerRevolution;
            }

            // correct for an overflow
            while (updatedPosition > std::numeric_limits<uint32_t>::max()) {
                updatedPosition -= mTicksPerRevolution;
            }

            return updatedPosition;
        }

        [[nodiscard]] auto offsetToRaw(int64_t position) const -> uint32_t {
            // apply the offset to the position of the servos
            int64_t updatedPosition = position + mPositionOffsetTicks;

            // correct for an underflow
            while (updatedPosition < 0) {
                updatedPosition += mTicksPerRevolution;
            }

            // correct for an overflow
            while (updatedPosition > std::numeric_limits<uint32_t>::max()) {
                updatedPosition -= mTicksPerRevolution;
            }

            return updatedPosition;
        }

        [[nodiscard]] auto getCurrentServoPosition() const -> std::pair<int64_t, U2D2::Status> {
            uint8_t hardwareStatus;
            uint32_t presentPosition;
            U2D2::Status const status = U2D2::getInstance()->read4Byte(ADDR_PRESENT_POSITION, presentPosition, mServoID, &hardwareStatus);

            int64_t offsetPosition = rawToOffset(presentPosition);

            RCLCPP_INFO_STREAM(mNode->get_logger(), "Current raw servo position " << presentPosition << " offset " << mPositionOffsetTicks << " offset position " << offsetPosition << " offset target " << mGoalPosition << " raw target position " << offsetToRaw(mGoalPosition));

            return std::make_pair(offsetPosition, status);
        }
    };
} // namespace mrover
