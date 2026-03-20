#include "servo.hpp"
#include "parameter.hpp"
#include <cstdint>

// Servo Addresses
static constexpr uint8_t ADDR_OPERATING_MODE = 11;
static constexpr uint8_t ADDR_TORQUE_ENABLE = 64;
static constexpr uint8_t ADDR_GOAL_POSITION = 116;
static constexpr uint8_t ADDR_PRESENT_POSITION = 132;
static constexpr uint8_t ADDR_PRESENT_VELOCITY = 128;
static constexpr uint8_t ADDR_PRESENT_CURRENT = 126;

#define SERVO_POSITION_DEAD_ZONE 5

#define SERVO_TICKS 4096

#define upper(val) ((val) == 0 ? 4096 : (val))

using namespace mrover;

auto Servo::updateConfigFromParameters() -> void {
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


    setProperty(ServoProperty::PositionPGain, static_cast<uint16_t>(positionPGain));
    setProperty(ServoProperty::PositionIGain, static_cast<uint16_t>(positionIGain));
    setProperty(ServoProperty::PositionDGain, static_cast<uint16_t>(positionDGain));
    setProperty(ServoProperty::VelocityPGain, static_cast<uint16_t>(velocityPGain));
    setProperty(ServoProperty::VelocityIGain, static_cast<uint16_t>(velocityIGain));
    setProperty(ServoProperty::CurrentLimit, static_cast<uint16_t>(currentLimit));
    setProperty(ServoProperty::ProfileAcceleration, static_cast<uint16_t>(profileAcceleration));
    setProperty(ServoProperty::ProfileVelocity, static_cast<uint16_t>(profileVelocity));

    ParameterWrapper::declareParameters(mNode.get(), parameters);

    mPositionMultiplier = static_cast<float>(positionMultiplier);

    int reverseLimitTicks = static_cast<int>((reverseLimit / 360.0f) * 4096.0f);
    int forwardLimitTicks = static_cast<int>((forwardLimit / 360.0f) * 4096.0f);

    mLimitAdjustment = (forwardLimitTicks + reverseLimitTicks) / 2;

    if (forwardLimitTicks > reverseLimitTicks) {
        mLimitAdjustment = (forwardLimitTicks + reverseLimitTicks + SERVO_TICKS) / 2;
    }
    mLimitAdjustment %= SERVO_TICKS;

    mAdjustedReverseLimit = (static_cast<int>((reverseLimit / 360.0f) * 4096.0f) - mLimitAdjustment + SERVO_TICKS) % SERVO_TICKS;
    mAdjustedForwardLimit = (static_cast<int>((forwardLimit / 360.0f) * 4096.0f) - mLimitAdjustment + SERVO_TICKS) % SERVO_TICKS;
}

Servo::Servo(rclcpp::Node::SharedPtr node, ServoId mServoId, std::string mServoName) : mServoId{mServoId}, mServoName{std::move(mServoName)},
                                                                                       mLimitAdjustment{0}, mAdjustedForwardLimit{0}, mAdjustedReverseLimit{0}, mGoalPosition{0}, mPositionMultiplier{0}, mNode{std::move(node)} {

    u2d2::registerServo(mServoId);
    updateConfigFromParameters();

    uint8_t hardwareStatus;

    // Use Position Control Mode
    u2d2::write1Byte(ADDR_OPERATING_MODE, 4, mServoId, &hardwareStatus);

    // Enable torque
    u2d2::write1Byte(ADDR_TORQUE_ENABLE, 1, mServoId, &hardwareStatus);
}

auto Servo::setPosition(ServoPosition position, ServoMode mode) -> u2d2::Status {
    // Convert degrees to ticks (0.0 - 360.0) to (0 to 4096)
    uint16_t targetPosition = static_cast<uint16_t>((position / 360.0f) * 4096.0f) % 4096;

    uint8_t hardwareStatus;

    uint32_t presentPosition;
    u2d2::read4Byte(ADDR_PRESENT_POSITION, presentPosition, mServoId, &hardwareStatus);
    presentPosition = static_cast<uint32_t>(static_cast<double>(presentPosition) / mPositionMultiplier);

    // Get current position (make sure its positive)
    uint16_t currentPosition = static_cast<uint16_t>((presentPosition + SERVO_TICKS) % SERVO_TICKS);

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
            int adjustedCurrentPosition = (currentPosition - mLimitAdjustment + SERVO_TICKS) % SERVO_TICKS;
            int adjustedTargetPosition = (targetPosition - mLimitAdjustment + SERVO_TICKS) % SERVO_TICKS;

            // If the current path to the final position goes over the middle limit, go the other way
            if (0 > adjustedCurrentPosition && 0 < adjustedTargetPosition) {

                if (normalizedDifference > 0)
                    normalizedDifference -= SERVO_TICKS;
                else if (normalizedDifference < 0)
                    normalizedDifference += SERVO_TICKS;
            }

            mAtLimit = false;

            // Limit destination if between mForwardLimit and middleLimit
            if (upper(adjustedTargetPosition) > mAdjustedForwardLimit) {
                normalizedDifference = (mAdjustedForwardLimit - adjustedCurrentPosition) % SERVO_TICKS;
                if (normalizedDifference < 0 && !(upper(adjustedCurrentPosition) > mAdjustedForwardLimit && adjustedCurrentPosition < SERVO_TICKS)) normalizedDifference += SERVO_TICKS;
                mAtLimit = true;
            }

            // Limit destination if between mReverseLimit and middleLimit
            else if (adjustedTargetPosition < upper(mAdjustedReverseLimit)) {

                normalizedDifference = (mAdjustedReverseLimit - adjustedCurrentPosition) % SERVO_TICKS;
                if (normalizedDifference > 0 && !(upper(adjustedCurrentPosition) > 0 && adjustedCurrentPosition < upper(mAdjustedReverseLimit))) normalizedDifference -= SERVO_TICKS;
                mAtLimit = true;
            }
        }
    }

    // Calculate final goal position
    mGoalPosition = currentPosition + normalizedDifference;
    // Write goal position
    return u2d2::write4Byte(ADDR_GOAL_POSITION, mGoalPosition * static_cast<uint32_t>(mPositionMultiplier), mServoId, &hardwareStatus);
}

auto Servo::getTargetStatus() const -> u2d2::Status {
    uint8_t hardwareStatus;
    uint32_t presentPosition;

    u2d2::Status status = u2d2::read4Byte(ADDR_PRESENT_POSITION, presentPosition, mServoId, &hardwareStatus);
    presentPosition = presentPosition / static_cast<uint32_t>(mPositionMultiplier);
    if (status != u2d2::Status::Success) return status;

    uint32_t mGoalPositionMod = mGoalPosition % SERVO_TICKS;

    if (presentPosition > mGoalPositionMod - SERVO_POSITION_DEAD_ZONE && presentPosition < mGoalPositionMod + SERVO_POSITION_DEAD_ZONE) {
        return u2d2::Status::Success;
    }

    if (hardwareStatus != 0) {
        return u2d2::Status::HardwareFailure;
    }

    return u2d2::Status::Active;
}

auto Servo::getPosition(ServoPosition& position) const -> u2d2::Status {
    uint8_t hardwareStatus;
    uint32_t positionInt;
    u2d2::Status status = u2d2::read4Byte(ADDR_PRESENT_POSITION, positionInt, mServoId, &hardwareStatus);
    positionInt = static_cast<uint32_t>(static_cast<double>(positionInt) / mPositionMultiplier);
    position = (static_cast<float>(positionInt % 4096) / 4096.0f) * 360.0f;
    return status;
}

auto Servo::getVelocity(ServoVelocity& velocity) const -> u2d2::Status {
    uint8_t hardwareStatus;
    uint32_t velocity_int;
    u2d2::Status status = u2d2::read4Byte(ADDR_PRESENT_VELOCITY, velocity_int, mServoId, &hardwareStatus);
    velocity = (static_cast<double>(velocity_int) * 0.22888); // 0.22888f Conversion factor to get rot/sec (found in dynamixel wizard)
    return status;
}

auto Servo::getCurrent(ServoCurrent& current) const -> u2d2::Status {
    uint8_t hardwareStatus;
    uint16_t currentInt;
    u2d2::Status status = u2d2::read2Byte(ADDR_PRESENT_CURRENT, currentInt, mServoId, &hardwareStatus);
    current = static_cast<double>(currentInt) / 1000.0;
    return status;
}

auto Servo::getPositionAbsolute(ServoPosition& position) const -> u2d2::Status {
    uint8_t hardwareStatus;
    uint32_t positionInt;
    u2d2::Status status = u2d2::read4Byte(ADDR_PRESENT_POSITION, positionInt, mServoId, &hardwareStatus);
    positionInt = static_cast<uint32_t>(static_cast<double>(positionInt) / mPositionMultiplier);
    position = (static_cast<double>(positionInt) / 4096.0f) * 360.0f;
    return status;
}

auto Servo::setProperty(ServoProperty prop, uint16_t value) -> u2d2::Status {
    uint8_t hardwareStatus;
    if (prop == ServoProperty::ProfileVelocity || prop == ServoProperty::ProfileAcceleration) {
        return u2d2::write4Byte(static_cast<ServoAddr>(prop), value, mServoId, &hardwareStatus);
    }
    return u2d2::write2Byte(static_cast<ServoAddr>(prop), value, mServoId, &hardwareStatus);
}


auto Servo::getLimitStatus() const -> bool {
    return mAtLimit;
}
