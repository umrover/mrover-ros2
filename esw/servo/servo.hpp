#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <parameter.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>

#include <u2d2.hpp>

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
        static constexpr uint8_t ADDR_GOAL_VELOCITY = 104;
        static constexpr uint8_t ADDR_GOAL_CURRENT = 102;

        static constexpr int32_t SERVO_TICKS = 4096;
        static constexpr uint8_t SERVO_POSITION_DEAD_ZONE = 5;
        static constexpr double SERVO_RPM_PER_TICK = 0.22888;
        static constexpr double SERVO_CURRENT_MA_PER_TICK = 2.69;

        static constexpr double TAU = 2 * M_PI;

        std::string mServoName{};
        int64_t mAdjustedForwardLimit{};
        int64_t mAdjustedReverseLimit{};
        int64_t mGoalPosition{};
        int64_t mPositionOffsetTicks{0};
        double mPositionMultiplier{1};
        uint8_t mServoID{};
        uint8_t mAtLimit{0};

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
            CurrentLimit = 38,
            VelocityLimit = 44,
            ProfileVelocity = 112,
            ProfileAcceleration = 108,
        };

        enum class ServoMode {
            Optimal,
            Clockwise,
            CounterClockwise,
            Limited,
        };

        enum class OperatingMode {
            Current = 0,
            Velocity = 1,
            Position = 3,
            ExtendedPosition = 4,
            CurrentBasedPosition = 5,
            PWM = 16
        };

    private:
        ServoMode mMode{ServoMode::Optimal};
        OperatingMode mOperatingMode{OperatingMode::Position};
        double mVelocityLimit{445.0};

        rclcpp::TimerBase::SharedPtr mWatchdogTimer;
        std::atomic<int64_t> mLastCommandTimeNs{0};
        std::atomic<bool> mVelocityTimedOut{false};
        static constexpr int64_t COMMAND_TIMEOUT_NS = 250'000'000;

    public:
        Servo(rclcpp::Node::SharedPtr node, std::string servoName) : mServoName{std::move(servoName)}, mPositionMultiplier{1}, mNode{std::move(node)} {

            mConfigPub = mNode->create_publisher<msg::ServoConfigure>("/u2d2/configure", rclcpp::QoS(10).transient_local());
            mCmdPub = mNode->create_publisher<msg::ServoIn>(std::format("/u2d2/{}/in", mServoName), 10);

            mStateSub = mNode->create_subscription<msg::ServoOut>(
                    std::format("/u2d2/{}/out", mServoName), 10,
                    [this](msg::ServoOut::ConstSharedPtr const& msg) -> void {
                        if (!mHasReceivedData) {
                            mPositionOffsetTicks = 0;
                            mHasReceivedData = true;
                        }
                        mCachedRawPosition = msg->position;
                        mCachedRawVelocity = msg->velocity;
                        mCachedRawCurrent = msg->current;
                        mCachedStatus = static_cast<U2D2::Status>(msg->status);
                    });

            int servoID;
            std::vector<ParameterWrapper> idParam = {{std::format("{}.id", mServoName), servoID, 0}};
            ParameterWrapper::declareParameters(mNode.get(), idParam);
            mServoID = static_cast<uint8_t>(servoID);

            rclcpp::sleep_for(std::chrono::milliseconds(200));
            msg::ServoConfigure configMessage;
            configMessage.name = mServoName;
            configMessage.id = mServoID;
            mConfigPub->publish(configMessage);
            rclcpp::sleep_for(std::chrono::milliseconds(500));

            publishWrite(ADDR_TORQUE_ENABLE, 1, 0);
            rclcpp::sleep_for(std::chrono::milliseconds(50));

            updateConfigFromParameters();
            publishWrite(ADDR_OPERATING_MODE, 1, static_cast<uint32_t>(mOperatingMode));
            publishWrite(ADDR_TORQUE_ENABLE, 1, 1);

            mLastCommandTimeNs = mNode->now().nanoseconds();

            mWatchdogTimer = mNode->create_wall_timer(
                std::chrono::milliseconds(100),
                [this]() -> void {
                    if (mOperatingMode == OperatingMode::Velocity) {
                        auto const now = mNode->now().nanoseconds();
                        if (!mVelocityTimedOut.load() && (now - mLastCommandTimeNs.load() > COMMAND_TIMEOUT_NS)) {
                            publishWrite(ADDR_GOAL_VELOCITY, 4, 0);
                            mVelocityTimedOut = true; 
                        }
                    }
                }
            );
        }

        auto setCurrentPosition(ServoPosition const position) -> U2D2::Status {
            auto const targetTicks = static_cast<int64_t>((position / TAU) * SERVO_TICKS);
            auto const currentRaw = static_cast<int32_t>(mCachedRawPosition.load());
            mPositionOffsetTicks = static_cast<int64_t>(currentRaw / mPositionMultiplier) - targetTicks;
            return U2D2::Status::Success;
        }

        auto getPosition(double& pos) const -> U2D2::Status {
            auto currentPositionAndStatus = getCurrentServoPosition();
            pos = (static_cast<double>(currentPositionAndStatus.first) / SERVO_TICKS) * TAU;
            return currentPositionAndStatus.second;
        }

        auto getVelocity(double& vel) const -> U2D2::Status {
            auto const rawVel = static_cast<int32_t>(mCachedRawVelocity.load());
            vel = (rawVel * SERVO_RPM_PER_TICK) * (TAU / 60.0);
            return mCachedStatus.load();
        }

        auto getCurrent(double& cur) const -> U2D2::Status {
            auto const rawCur = static_cast<int16_t>(mCachedRawCurrent.load());
            if (std::abs(rawCur) < 12) cur = 0.0;
            else cur = (rawCur * SERVO_CURRENT_MA_PER_TICK) / 1000.0;
            return mCachedStatus.load();
        }

        auto setPosition(ServoPosition const position) -> U2D2::Status {
            mGoalPosition = static_cast<int64_t>((position / TAU) * static_cast<double>(SERVO_TICKS));

            auto currentPositionAndStatus = getCurrentServoPosition();

            if (currentPositionAndStatus.second != U2D2::Status::Success) {
                return currentPositionAndStatus.second;
            }

            mAtLimit = 0; 

            if (mMode == ServoMode::Limited) {
                if (!isWithinLimits(mGoalPosition)) {
                    mGoalPosition = clampToLimits(mGoalPosition);
                }
                publishWrite(ADDR_GOAL_POSITION, 4, offsetToRaw(mGoalPosition));
            } else {
                auto const targetRaw = static_cast<int64_t>((mGoalPosition + mPositionOffsetTicks) * mPositionMultiplier);
                auto const currentRaw = static_cast<int64_t>(mCachedRawPosition.load());

                int64_t diff = (targetRaw - currentRaw) % SERVO_TICKS;
                if (diff < 0) diff += SERVO_TICKS;

                if (mMode == ServoMode::Optimal) {
                    if (diff > (SERVO_TICKS / 2)) {
                        diff -= SERVO_TICKS;
                    }
                } else if (mMode == ServoMode::Clockwise) {
                    if (diff > 0) diff -= SERVO_TICKS;
                }

                int64_t newTargetRaw = currentRaw + diff;
                mGoalPosition = static_cast<int64_t>(newTargetRaw / mPositionMultiplier) - mPositionOffsetTicks;
                publishWrite(ADDR_GOAL_POSITION, 4, static_cast<uint32_t>(static_cast<int32_t>(newTargetRaw)));
            }

            return U2D2::Status::Success;
        }

        auto setVelocityTarget(ServoVelocity const velocity) -> U2D2::Status {
            mLastCommandTimeNs = mNode->now().nanoseconds();
            mVelocityTimedOut = false;

            double const rpm = velocity * (60.0 / TAU);
            auto const velocityTicks = static_cast<int32_t>(rpm / SERVO_RPM_PER_TICK);

            publishWrite(ADDR_GOAL_VELOCITY, 4, static_cast<uint32_t>(velocityTicks));
            return U2D2::Status::Success;
        }

        [[nodiscard]] auto setProperty(ServoProperty prop, uint32_t const value) const -> U2D2::Status {
            uint8_t const len = (prop == ServoProperty::ProfileVelocity || prop == ServoProperty::ProfileAcceleration || prop == ServoProperty::VelocityLimit) ? 4 : 2;
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

        [[nodiscard]] auto getLimitStatus() const -> uint8_t { return mAtLimit; }

        [[nodiscard]] auto getVelocityLimitRadPerSec() const -> double {
            double const rpm = mVelocityLimit * SERVO_RPM_PER_TICK;
            return rpm * (TAU / 60.0);
        }

    private:
        auto publishWrite(uint8_t addr, uint8_t len, uint32_t val) const -> void {
            msg::ServoIn msg;
            msg.addr = addr;
            msg.length = len;
            msg.value = val;
            mCmdPub->publish(msg);
        }

        auto updateConfigFromParameters() -> void {
            std::string modeString;
            std::string operatingModeStr;
            double forwardLimit;
            double reverseLimit;
            double positionPGain;
            double positionIGain;
            double positionDGain;
            double velocityPGain;
            double velocityIGain;
            double currentLimit;
            double velocityLimit;
            double profileAcceleration;
            double profileVelocity;

            std::vector<ParameterWrapper> parameters = {
                    {std::format("{}.mode", mServoName), modeString, std::string("optimal")},
                    {std::format("{}.operating_mode", mServoName), operatingModeStr, std::string("position")},
                    {std::format("{}.position_multiplier", mServoName), mPositionMultiplier, 1.0},
                    {std::format("{}.reverse_limit", mServoName), reverseLimit, 0.0},
                    {std::format("{}.forward_limit", mServoName), forwardLimit, TAU}, 
                    {std::format("{}.position_p", mServoName), positionPGain, 400.0},
                    {std::format("{}.position_i", mServoName), positionIGain, 0.0},
                    {std::format("{}.position_d", mServoName), positionDGain, 0.0},
                    {std::format("{}.velocity_p", mServoName), velocityPGain, 180.0},
                    {std::format("{}.velocity_i", mServoName), velocityIGain, 90.0},
                    {std::format("{}.current_limit", mServoName), currentLimit, 1750.0},
                    {std::format("{}.velocity_limit", mServoName), velocityLimit, 445.0},
                    {std::format("{}.profile_acceleration", mServoName), profileAcceleration, 100.0},
                    {std::format("{}.profile_velocity", mServoName), profileVelocity, 100.0}};

            ParameterWrapper::declareParameters(mNode.get(), parameters);

            if (modeString == "clockwise") mMode = ServoMode::Clockwise;
            else if (modeString == "counter_clockwise") mMode = ServoMode::CounterClockwise;
            else if (modeString == "limited") mMode = ServoMode::Limited;
            else mMode = ServoMode::Optimal;

            if (operatingModeStr == "velocity") mOperatingMode = OperatingMode::Velocity;
            else if (operatingModeStr == "extended_position") mOperatingMode = OperatingMode::ExtendedPosition;
            else if (operatingModeStr == "current") mOperatingMode = OperatingMode::Current;
            else if (operatingModeStr == "pwm") mOperatingMode = OperatingMode::PWM;
            else mOperatingMode = OperatingMode::Position;

            mVelocityLimit = velocityLimit;

            (void) setProperty(ServoProperty::PositionPGain, static_cast<uint32_t>(positionPGain));
            (void) setProperty(ServoProperty::PositionIGain, static_cast<uint32_t>(positionIGain));
            (void) setProperty(ServoProperty::PositionDGain, static_cast<uint32_t>(positionDGain));
            (void) setProperty(ServoProperty::VelocityPGain, static_cast<uint32_t>(velocityPGain));
            (void) setProperty(ServoProperty::VelocityIGain, static_cast<uint32_t>(velocityIGain));
            (void) setProperty(ServoProperty::CurrentLimit, static_cast<uint32_t>(currentLimit));
            (void) setProperty(ServoProperty::VelocityLimit, static_cast<uint32_t>(mVelocityLimit));
            (void) setProperty(ServoProperty::ProfileAcceleration, static_cast<uint32_t>(profileAcceleration));
            (void) setProperty(ServoProperty::ProfileVelocity, static_cast<uint32_t>(profileVelocity));

            mAdjustedReverseLimit = static_cast<int64_t>((reverseLimit / TAU) * SERVO_TICKS);
            mAdjustedForwardLimit = static_cast<int64_t>((forwardLimit / TAU) * SERVO_TICKS);
        }

        [[nodiscard]] auto isWithinLimits(int64_t targetTicks) const -> bool {
            if (mAdjustedReverseLimit <= mAdjustedForwardLimit) {
                return targetTicks >= mAdjustedReverseLimit && targetTicks <= mAdjustedForwardLimit;
            } else {
                int64_t modTarget = targetTicks % SERVO_TICKS;
                if (modTarget < 0) modTarget += SERVO_TICKS;
                int64_t modRev = mAdjustedReverseLimit % SERVO_TICKS;
                int64_t modFwd = mAdjustedForwardLimit % SERVO_TICKS;
                return modTarget >= modRev || modTarget <= modFwd;
            }
        }

        [[nodiscard]] auto clampToLimits(int64_t targetTicks) -> int64_t {
            if (mAdjustedReverseLimit <= mAdjustedForwardLimit) {
                if (targetTicks > mAdjustedForwardLimit) {
                    mAtLimit |= 0x01;
                    return mAdjustedForwardLimit;
                }
                if (targetTicks < mAdjustedReverseLimit) {
                    mAtLimit |= 0x02;
                    return mAdjustedReverseLimit;
                }
                return targetTicks;
            } else {
                int64_t modTarget = targetTicks % SERVO_TICKS;
                if (modTarget < 0) modTarget += SERVO_TICKS;

                auto getDistance = [](int64_t from, int64_t to) -> int64_t {
                    int64_t diff = std::abs(from - to) % SERVO_TICKS;
                    return diff > (SERVO_TICKS / 2) ? SERVO_TICKS - diff : diff;
                };

                if (getDistance(modTarget, mAdjustedForwardLimit) < getDistance(modTarget, mAdjustedReverseLimit)) {
                    mAtLimit |= 0x01;
                    return targetTicks + getDistance(modTarget, mAdjustedForwardLimit); 
                } else {
                    mAtLimit |= 0x02;
                    return targetTicks - getDistance(modTarget, mAdjustedReverseLimit);
                }
            }
        }

        [[nodiscard]] auto rawToOffset(uint32_t position) const -> int64_t {
            auto const signedPos = static_cast<int32_t>(position);
            return static_cast<int64_t>(signedPos / mPositionMultiplier) - mPositionOffsetTicks;
        }

        [[nodiscard]] auto offsetToRaw(int64_t position) const -> uint32_t {
            auto const rawTarget = static_cast<int64_t>((position + mPositionOffsetTicks) * mPositionMultiplier);
            return static_cast<uint32_t>(static_cast<int32_t>(rawTarget));
        }

        [[nodiscard]] auto getCurrentServoPosition() const -> std::pair<int64_t, U2D2::Status> {
            int64_t offsetPosition = rawToOffset(mCachedRawPosition.load());
            return std::make_pair(offsetPosition, mCachedStatus.load());
        }
    };
} // namespace mrover
