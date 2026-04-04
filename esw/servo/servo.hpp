#pragma once

#include <atomic>
#include <cstdint>
#include <cmath>
#include <parameter.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>

#include "u2d2.hpp"
#include <mrover/msg/servo_configure.hpp>
#include <mrover/msg/servo_in.hpp>
#include <mrover/msg/servo_out.hpp>

namespace mrover {

    class Servo {
        using ServoID = uint8_t;
        using ServoPosition = double;
        using ServoVelocity = double;
        using ServoCurrent = double;
        using ServoAddr = uint16_t;

        static constexpr uint8_t ADDR_OPERATING_MODE = 11;
        static constexpr uint8_t ADDR_TORQUE_ENABLE = 64;
        static constexpr uint8_t ADDR_GOAL_POSITION = 116;

        static constexpr int32_t SERVO_TICKS = 4096;
        static constexpr uint8_t SERVO_POSITION_DEAD_ZONE = 5;

        static constexpr double TAU = 2 * M_PI;

        [[nodiscard]] constexpr auto getUpper(int64_t const val) const -> auto { return val == 0 ? SERVO_TICKS : val; }

        std::string mServoName;
        int64_t mLimitAdjustment;
        int64_t mAdjustedForwardLimit;
        int64_t mAdjustedReverseLimit;
        int64_t mGoalPosition;
        uint32_t mPositionOffsetTicks;
        double mPositionMultiplier;

        bool mAtLimit = false;
        uint8_t mServoID;

        rclcpp::Node::SharedPtr mNode;
        rclcpp::Publisher<msg::ServoConfigure>::SharedPtr mConfigPub;
        rclcpp::Publisher<msg::ServoIn>::SharedPtr mCmdPub;
        rclcpp::Subscription<msg::ServoOut>::SharedPtr mStateSub;

        std::atomic<bool> mHasReceivedData{false};
        std::atomic<uint32_t> mCachedRawPosition{0};
        std::atomic<uint32_t> mCachedRawVelocity{0};
        std::atomic<uint16_t> mCachedRawCurrent{0};
        std::atomic<U2D2::Status> mCachedStatus{U2D2::Status::CommRxWaiting};

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

        enum class ServoMode { Optimal,
                               Clockwise,
                               CounterClockwise,
                               Limited };

        Servo(rclcpp::Node::SharedPtr node, std::string servoName) : mServoName{std::move(servoName)}, mLimitAdjustment{0}, mAdjustedForwardLimit{0},
                                                                     mAdjustedReverseLimit{0}, mGoalPosition{0}, mPositionOffsetTicks{0}, mPositionMultiplier{1}, mNode{std::move(node)} {

            mConfigPub = mNode->create_publisher<msg::ServoConfigure>("/u2d2/configure", rclcpp::QoS(10).transient_local()); // transient so messages queued if published before subscribed
            mCmdPub = mNode->create_publisher<msg::ServoIn>(std::format("/u2d2/{}/in", mServoName), 10);

            mStateSub = mNode->create_subscription<msg::ServoOut>(
                    std::format("/u2d2/{}/out", mServoName), 10,
                    [this](msg::ServoOut::ConstSharedPtr const& msg) {
                        if (!mHasReceivedData) {
                            mPositionOffsetTicks = msg->position;
                            mHasReceivedData = true;
                        }
                        mCachedRawPosition = msg->position;
                        mCachedRawVelocity = msg->velocity;
                        mCachedRawCurrent = msg->current;
                        mCachedStatus = static_cast<U2D2::Status>(msg->status);
                    });

            updateConfigFromParameters();

            rclcpp::sleep_for(std::chrono::milliseconds(200));

            msg::ServoConfigure configMessage;
            configMessage.name = mServoName;
            configMessage.id = mServoID;
            mConfigPub->publish(configMessage);

            rclcpp::sleep_for(std::chrono::milliseconds(200));

            publishWrite(ADDR_OPERATING_MODE, 1, 4); // Position Control Mode
            publishWrite(ADDR_TORQUE_ENABLE, 1, 1);  // Enable torque
        }

        auto setPosition(ServoPosition const position, ServoMode const mode) -> U2D2::Status {
            // Convert degrees to ticks (0.0 - 360.0) to (0 to SERVO_TICKS)
            mGoalPosition = static_cast<int64_t>((position / TAU) * static_cast<double>(SERVO_TICKS));


            auto currentPositionAndStatus = getCurrentServoPosition();

            if (currentPositionAndStatus.second != U2D2::Status::Success) {
                return currentPositionAndStatus.second;
            }

            auto normalizedDifference = static_cast<int64_t>(mGoalPosition - currentPositionAndStatus.first);
            mAtLimit = false;

            switch (mode) {
                case ServoMode::Optimal:
                    if (normalizedDifference > (SERVO_TICKS / 2)) {
                        mGoalPosition -= SERVO_TICKS; // Go the other (shorter) way around
                    } else if (normalizedDifference < -(SERVO_TICKS / 2)) {
                        mGoalPosition += SERVO_TICKS; // Go the other (shorter) way around
                    }
                    break;
                case ServoMode::Clockwise: // clockwise
                    if (normalizedDifference < 0) {
                        mGoalPosition += SERVO_TICKS;
                    }
                    break;
                case ServoMode::CounterClockwise: // counter clockwise
                    if (normalizedDifference > 0) {
                        mGoalPosition -= SERVO_TICKS;
                    }
                    break;
                case ServoMode::Limited: {

                    // Adjust target and current position
                    int64_t const adjustedCurrentPosition = (currentPositionAndStatus.first - mLimitAdjustment + SERVO_TICKS) % SERVO_TICKS;
                    int64_t const adjustedTargetPosition = (mGoalPosition - mLimitAdjustment + SERVO_TICKS) % SERVO_TICKS;

                    if (0 > adjustedCurrentPosition && 0 < adjustedTargetPosition) {
                        if (normalizedDifference > 0)
                            mGoalPosition -= SERVO_TICKS;
                        else if (normalizedDifference < 0)
                            mGoalPosition += SERVO_TICKS;
                    }

                    mAtLimit = false;
                    if (getUpper(adjustedTargetPosition) > mAdjustedForwardLimit) {
                        mGoalPosition = (mAdjustedForwardLimit - adjustedCurrentPosition) % SERVO_TICKS;
                        if (normalizedDifference < 0 && !(getUpper(adjustedCurrentPosition) > mAdjustedForwardLimit && adjustedCurrentPosition < SERVO_TICKS)) mGoalPosition += SERVO_TICKS;
                        mAtLimit = true;
                    }

                    // Limit destination if between mReverseLimit and middleLimit
                    else if (adjustedTargetPosition < getUpper(mAdjustedReverseLimit)) {

                        mGoalPosition = (mAdjustedReverseLimit - adjustedCurrentPosition) % SERVO_TICKS;
                        if (normalizedDifference > 0 && !(getUpper(adjustedCurrentPosition) > 0 && adjustedCurrentPosition < getUpper(mAdjustedReverseLimit))) mGoalPosition -= SERVO_TICKS;
                        mAtLimit = true;
                    }
                }
            }

            publishWrite(ADDR_GOAL_POSITION, 4, offsetToRaw(mGoalPosition));
            return U2D2::Status::Success;
        }

        auto getPosition(ServoPosition& position) const -> U2D2::Status {
            auto const positionTicks = getCurrentServoPosition();
            position = (static_cast<double>(positionTicks.first) / static_cast<double>(SERVO_TICKS)) * TAU;
            return positionTicks.second;
        }

        auto getVelocity(ServoVelocity& velocity) const -> U2D2::Status {
            velocity = (static_cast<double>(mCachedRawVelocity.load()) * 0.22888);
            return mCachedStatus.load();
        }

        auto getCurrent(ServoCurrent& current) const -> U2D2::Status {
            current = static_cast<double>(mCachedRawCurrent.load()) / 1000.0;
            return mCachedStatus.load();
        }

        auto setProperty(ServoProperty prop, uint16_t const value) const -> U2D2::Status {
            uint8_t const len = (prop == ServoProperty::ProfileVelocity || prop == ServoProperty::ProfileAcceleration) ? 4 : 2;
            publishWrite(static_cast<ServoAddr>(prop), len, value);
            return U2D2::Status::Success;
        }

        [[nodiscard]] auto getTargetStatus() const -> U2D2::Status {
            if (!mHasReceivedData) return U2D2::Status::CommRxWaiting;
            auto const currentPositionAndStatus = getCurrentServoPosition();
            if (currentPositionAndStatus.second != U2D2::Status::Success) return currentPositionAndStatus.second;
            if (std::abs(currentPositionAndStatus.first - mGoalPosition) < SERVO_POSITION_DEAD_ZONE) {
                return U2D2::Status::Success;
            }
            return U2D2::Status::Active;
        }

        [[nodiscard]] auto getLimitStatus() const -> bool { return mAtLimit; }

    private:
        auto publishWrite(uint8_t addr, uint8_t len, uint32_t val) const -> void {
            msg::ServoIn msg;
            msg.addr = addr;
            msg.length = len;
            msg.value = val;
            mCmdPub->publish(msg);
        }

        auto updateConfigFromParameters() -> void {
            int servoID;
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

            std::vector<ParameterWrapper> parameters = {
                    {std::format("{}.id", mServoName), servoID, 0},
                    {std::format("{}.position_multiplier", mServoName), mPositionMultiplier, 1.0},
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

            mServoID = static_cast<uint8_t>(servoID);

            setProperty(ServoProperty::PositionPGain, static_cast<uint16_t>(positionPGain));
            setProperty(ServoProperty::PositionIGain, static_cast<uint16_t>(positionIGain));
            setProperty(ServoProperty::PositionDGain, static_cast<uint16_t>(positionDGain));
            setProperty(ServoProperty::VelocityPGain, static_cast<uint16_t>(velocityPGain));
            setProperty(ServoProperty::VelocityIGain, static_cast<uint16_t>(velocityIGain));
            setProperty(ServoProperty::CurrentLimit, static_cast<uint16_t>(currentLimit));
            setProperty(ServoProperty::ProfileAcceleration, static_cast<uint16_t>(profileAcceleration));
            setProperty(ServoProperty::ProfileVelocity, static_cast<uint16_t>(profileVelocity));

            int const reverseLimitTicks = static_cast<int>((reverseLimit / TAU) * SERVO_TICKS);
            int const forwardLimitTicks = static_cast<int>((forwardLimit / TAU) * SERVO_TICKS);

            mLimitAdjustment = (forwardLimitTicks + reverseLimitTicks) / 2;
            if (forwardLimitTicks > reverseLimitTicks) {
                mLimitAdjustment = (forwardLimitTicks + reverseLimitTicks + SERVO_TICKS) / 2;
            }
            mLimitAdjustment %= SERVO_TICKS;

            mAdjustedReverseLimit = (static_cast<int64_t>((reverseLimit / TAU) * SERVO_TICKS) - mLimitAdjustment + SERVO_TICKS) % SERVO_TICKS;
            mAdjustedForwardLimit = (static_cast<int>((forwardLimit / TAU) * static_cast<double>(SERVO_TICKS)) - mLimitAdjustment + SERVO_TICKS) % SERVO_TICKS;
        }

        [[nodiscard]] auto rawToOffset(uint32_t position) const -> int64_t {
            // apply the offset to the position of the servos
            int64_t updatedPosition = static_cast<int64_t>(position / mPositionMultiplier) - mPositionOffsetTicks;

            // correct for an underflow
            while (updatedPosition < 0) {
                updatedPosition += SERVO_TICKS;
            }

            // correct for an overflow
            while (updatedPosition > std::numeric_limits<uint32_t>::max()) {
                updatedPosition -= SERVO_TICKS;
            }

            return updatedPosition;
        }

        [[nodiscard]] auto offsetToRaw(int64_t position) const -> uint32_t {
            // apply the offset to the position of the servos
            int64_t updatedPosition = static_cast<int64_t>(static_cast<double>(position) * mPositionMultiplier) + mPositionOffsetTicks;

            // correct for an underflow
            while (updatedPosition < 0) {
                updatedPosition += SERVO_TICKS;
            }

            // correct for an overflow
            while (updatedPosition > std::numeric_limits<uint32_t>::max()) {
                updatedPosition -= SERVO_TICKS;
            }

            return updatedPosition;
        }

        [[nodiscard]] auto getCurrentServoPosition() const -> std::pair<int64_t, U2D2::Status> {
            int64_t offsetPosition = rawToOffset(mCachedRawPosition.load());
            return std::make_pair(offsetPosition, mCachedStatus.load());
        }
    };
} // namespace mrover
