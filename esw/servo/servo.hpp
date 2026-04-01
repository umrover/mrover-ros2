#pragma once

#include <parameter.hpp>
#include <rclcpp/logging.hpp>
#include <string>

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

        static constexpr int getUpper(int const val) { return val == 0 ? 4096 : val; }

        ServoID mServoID;
        std::string mServoName;
        int mLimitAdjustment;
        int mAdjustedForwardLimit;
        int mAdjustedReverseLimit;
        uint32_t mGoalPosition;
        float mPositionMultiplier;

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
                                                                     mAdjustedReverseLimit{0}, mGoalPosition{0}, mPositionMultiplier{0}, mNode{std::move(node)} {

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
            // Convert degrees to ticks (0.0 - 360.0) to (0 to 4096)
            auto const targetPosition = static_cast<uint16_t>((position / 360.0f) * 4096.0f) % 4096;

            uint8_t hardwareStatus;

            uint32_t presentPosition;
            U2D2::getInstance()->read4Byte(ADDR_PRESENT_POSITION, presentPosition, mServoID, &hardwareStatus);
            presentPosition = static_cast<uint32_t>(static_cast<double>(presentPosition) / mPositionMultiplier);

            // Get current position (make sure its positive)
            auto const currentPosition = static_cast<uint16_t>((presentPosition + SERVO_TICKS) % SERVO_TICKS);

            // Calculate the signed difference (accounting for overflow)
            int normalizedDifference = static_cast<int>(targetPosition - currentPosition);

            mAtLimit = false;

            switch (mode) {
                case ServoMode::Optimal:
                    if (normalizedDifference > (SERVO_TICKS / 2)) {
                        normalizedDifference -= SERVO_TICKS; // Go the other (shorter) way around
                    } else if (normalizedDifference < -(SERVO_TICKS / 2)) {
                        normalizedDifference += SERVO_TICKS; // Go the other (shorter) way around
                    }
                    break;
                case ServoMode::Clockwise: // clockwise
                    if (normalizedDifference < 0) {
                        normalizedDifference += SERVO_TICKS;
                    }
                    break;
                case ServoMode::CounterClockwise: // counter clockwise
                    if (normalizedDifference > 0) {
                        normalizedDifference -= SERVO_TICKS;
                    }
                    break;
                case ServoMode::Limited: {

                    // Adjust target and current position
                    int const adjustedCurrentPosition = (currentPosition - mLimitAdjustment + SERVO_TICKS) % SERVO_TICKS;
                    int const adjustedTargetPosition = (targetPosition - mLimitAdjustment + SERVO_TICKS) % SERVO_TICKS;

                    // If the current path to the final position goes over the middle limit, go the other way
                    if (0 > adjustedCurrentPosition && 0 < adjustedTargetPosition) {

                        if (normalizedDifference > 0)
                            normalizedDifference -= SERVO_TICKS;
                        else if (normalizedDifference < 0)
                            normalizedDifference += SERVO_TICKS;
                    }

                    mAtLimit = false;

                    // Limit destination if between mForwardLimit and middleLimit
                    if (getUpper(adjustedTargetPosition) > mAdjustedForwardLimit) {
                        normalizedDifference = (mAdjustedForwardLimit - adjustedCurrentPosition) % SERVO_TICKS;
                        if (normalizedDifference < 0 && !(getUpper(adjustedCurrentPosition) > mAdjustedForwardLimit && adjustedCurrentPosition < SERVO_TICKS)) normalizedDifference += SERVO_TICKS;
                        mAtLimit = true;
                    }

                    // Limit destination if between mReverseLimit and middleLimit
                    else if (adjustedTargetPosition < getUpper(mAdjustedReverseLimit)) {

                        normalizedDifference = (mAdjustedReverseLimit - adjustedCurrentPosition) % SERVO_TICKS;
                        if (normalizedDifference > 0 && !(getUpper(adjustedCurrentPosition) > 0 && adjustedCurrentPosition < getUpper(mAdjustedReverseLimit))) normalizedDifference -= SERVO_TICKS;
                        mAtLimit = true;
                    }
                }
            }

            // Calculate final goal position
            mGoalPosition = currentPosition + normalizedDifference;
            // Write goal position
            return U2D2::getInstance()->write4Byte(ADDR_GOAL_POSITION, static_cast<uint32_t>(static_cast<float>(mGoalPosition) * mPositionMultiplier), mServoID, &hardwareStatus);
        }


        auto getPosition(ServoPosition& position) const -> U2D2::Status {
            uint8_t hardwareStatus;
            uint32_t positionInt;
            U2D2::Status const status = U2D2::getInstance()->read4Byte(ADDR_PRESENT_POSITION, positionInt, mServoID, &hardwareStatus);
            positionInt = static_cast<uint32_t>(static_cast<double>(positionInt) / mPositionMultiplier);
            position = (static_cast<float>(positionInt % 4096) / 4096.0f) * 360.0f;
            return status;
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

        auto getPositionAbsolute(ServoPosition& position) const -> U2D2::Status {
            uint8_t hardwareStatus;
            uint32_t positionInt;
            U2D2::Status const status = U2D2::getInstance()->read4Byte(ADDR_PRESENT_POSITION, positionInt, mServoID, &hardwareStatus);
            positionInt = static_cast<uint32_t>(static_cast<double>(positionInt) / mPositionMultiplier);
            position = (static_cast<double>(positionInt) / 4096.0f) * 360.0f;
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
            uint8_t hardwareStatus;
            uint32_t presentPosition;

            U2D2::Status const status = U2D2::getInstance()->read4Byte(ADDR_PRESENT_POSITION, presentPosition, mServoID, &hardwareStatus);
            presentPosition = presentPosition / static_cast<uint32_t>(mPositionMultiplier);
            if (status != U2D2::Status::Success) return status;

            if (uint32_t const mGoalPositionMod = mGoalPosition % SERVO_TICKS; presentPosition > mGoalPositionMod - SERVO_POSITION_DEAD_ZONE && presentPosition < mGoalPositionMod + SERVO_POSITION_DEAD_ZONE) {
                return U2D2::Status::Success;
            }

            if (hardwareStatus != 0) {
                return U2D2::Status::HardwareFailure;
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
            double positionMultiplier;
            double profileAcceleration;
            double profileVelocity;

            std::vector<ParameterWrapper> parameters = {
                    {std::format("{}.position_multiplier", mServoName), positionMultiplier, 340.0},
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

            mNode->get_parameter(std::format("{}.position_multiplier", mServoName), positionMultiplier);
            mNode->get_parameter(std::format("{}.reverse_limit", mServoName), reverseLimit);
            mNode->get_parameter(std::format("{}.forward_limit", mServoName), forwardLimit);
            mNode->get_parameter(std::format("{}.position_p", mServoName), positionPGain);
            mNode->get_parameter(std::format("{}.position_i", mServoName), positionIGain);
            mNode->get_parameter(std::format("{}.position_d", mServoName), positionDGain);
            mNode->get_parameter(std::format("{}.velocity_p", mServoName), velocityPGain);
            mNode->get_parameter(std::format("{}.velocity_i", mServoName), velocityIGain);
            mNode->get_parameter(std::format("{}.current_limit", mServoName), currentLimit);
            mNode->get_parameter(std::format("{}.profile_acceleration", mServoName), profileAcceleration);
            mNode->get_parameter(std::format("{}.profile_velocity", mServoName), profileVelocity);

            check(setProperty(ServoProperty::PositionPGain, static_cast<uint16_t>(positionPGain)) == U2D2::Status::Success, "pos p gain error");
            check(setProperty(ServoProperty::PositionIGain, static_cast<uint16_t>(positionIGain)) == U2D2::Status::Success, "pos i gain error");
            check(setProperty(ServoProperty::PositionDGain, static_cast<uint16_t>(positionDGain)) == U2D2::Status::Success, "pos d gain error");
            check(setProperty(ServoProperty::VelocityPGain, static_cast<uint16_t>(velocityPGain)) == U2D2::Status::Success, "vel p gain error");
            check(setProperty(ServoProperty::VelocityIGain, static_cast<uint16_t>(velocityIGain)) == U2D2::Status::Success, "vel i gain error");
            check(setProperty(ServoProperty::CurrentLimit, static_cast<uint16_t>(currentLimit)) == U2D2::Status::Success, "current limit gain error");
            check(setProperty(ServoProperty::ProfileAcceleration, static_cast<uint16_t>(profileAcceleration)) == U2D2::Status::Success, "profile accel gain error");
            check(setProperty(ServoProperty::ProfileVelocity, static_cast<uint16_t>(profileVelocity)) == U2D2::Status::Success, "profile vel gain error");

            ParameterWrapper::declareParameters(mNode.get(), parameters);

            mPositionMultiplier = static_cast<float>(positionMultiplier);

            int const reverseLimitTicks = static_cast<int>((reverseLimit / 360.0f) * 4096.0f);
            int const forwardLimitTicks = static_cast<int>((forwardLimit / 360.0f) * 4096.0f);

            mLimitAdjustment = (forwardLimitTicks + reverseLimitTicks) / 2;

            if (forwardLimitTicks > reverseLimitTicks) {
                mLimitAdjustment = (forwardLimitTicks + reverseLimitTicks + SERVO_TICKS) / 2;
            }
            mLimitAdjustment %= SERVO_TICKS;

            mAdjustedReverseLimit = (static_cast<int>((reverseLimit / 360.0f) * 4096.0f) - mLimitAdjustment + SERVO_TICKS) % SERVO_TICKS;
            mAdjustedForwardLimit = (static_cast<int>((forwardLimit / 360.0f) * 4096.0f) - mLimitAdjustment + SERVO_TICKS) % SERVO_TICKS;
        }
    };
} // namespace mrover
