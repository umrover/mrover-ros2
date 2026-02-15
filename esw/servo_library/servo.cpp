#include "servo.hpp"
#include "parameter.hpp"
#include <cstdint>

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.
#define SERVO_BAUDRATE 57600 // Baud rate of dynamixel devices

// Servo Addresses
static constexpr uint8_t ADDR_OPERATING_MODE = 11;
static constexpr uint8_t ADDR_TORQUE_ENABLE = 64;
static constexpr uint8_t ADDR_GOAL_POSITION = 116;
static constexpr uint8_t ADDR_PRESENT_POSITION = 132;
static constexpr uint8_t ADDR_PRESENT_VELOCITY = 128;
static constexpr uint8_t ADDR_PRESENT_CURRENT = 126;
static constexpr uint8_t ADDR_CURRENT_LIMIT = 102;
static constexpr uint8_t ADDR_VELOCITY_I_GAIN = 76;
static constexpr uint8_t ADDR_VELOCITY_P_GAIN = 78;
static constexpr uint8_t ADDR_POSITION_D_GAIN = 80;
static constexpr uint8_t ADDR_POSITION_I_GAIN = 82;
static constexpr uint8_t ADDR_POSITION_P_GAIN = 84;

#define SERVO_POSITION_DEAD_ZONE 5

#define SERVO_TICKS 4096

#define upper(val) (val == 0 ? 4096 : val)

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

    std::vector<ParameterWrapper> parameters = {
            {std::format("{}.reverse_limit", mServoName), forwardLimit, 90.0},
            {std::format("{}.forward_limit", mServoName), reverseLimit, 270.0},
            {std::format("{}.position_p", mServoName), positionPGain, 400.0},
            {std::format("{}.position_i", mServoName), positionIGain, 0.0},
            {std::format("{}.position_d", mServoName), positionDGain, 0.0},
            {std::format("{}.velocity_p", mServoName), velocityPGain, 180.0},
            {std::format("{}.velocity_i", mServoName), velocityIGain, 90.0},
            {std::format("{}.current_limit", mServoName), currentLimit, 1750.0}
    };

    ParameterWrapper::declareParameters(mNode.get(), parameters);

    mNode->get_parameter(std::format("{}.reverse_limit", mServoName), forwardLimit);
    mNode->get_parameter(std::format("{}.forward_limit", mServoName), reverseLimit);
    mNode->get_parameter(std::format("{}.position_p", mServoName), positionPGain);
    mNode->get_parameter(std::format("{}.position_i", mServoName), positionIGain);
    mNode->get_parameter(std::format("{}.position_d", mServoName), positionDGain);
    mNode->get_parameter(std::format("{}.velocity_p", mServoName), velocityPGain);
    mNode->get_parameter(std::format("{}.velocity_i", mServoName), velocityIGain);
    mNode->get_parameter(std::format("{}.current_limit", mServoName), currentLimit);
    
    setProperty(ServoProperty::PositionPGain, static_cast<uint16_t>(positionPGain));
    setProperty(ServoProperty::PositionIGain, static_cast<uint16_t>(positionIGain));
    setProperty(ServoProperty::PositionDGain, static_cast<uint16_t>(positionDGain));
    setProperty(ServoProperty::VelocityPGain, static_cast<uint16_t>(velocityPGain));
    setProperty(ServoProperty::VelocityIGain, static_cast<uint16_t>(velocityIGain));
    setProperty(ServoProperty::CurrentLimit, static_cast<uint16_t>(currentLimit));

    ParameterWrapper::declareParameters(mNode.get(), parameters);

    
    mReverseLimit = static_cast<int>((static_cast<float>(90) / 360.0f) * 4096.0f);
    mForwardLimit = static_cast<int>((static_cast<float>(270) / 360.0f) * 4096.0f);

    assert(mForwardLimit >= 0);
    assert(mReverseLimit >= 0);
    assert(mForwardLimit <= 4096);
    assert(mReverseLimit <= 4096);
}

Servo::Servo(rclcpp::Node::SharedPtr node, ServoId mServoId, std::string mServoName) : 
        mServoId{mServoId}, mServoName{std::move(mServoName)},
        mForwardLimit{0}, mReverseLimit{0}, mGoalPosition{0}, mNode{std::move(node)}
{
    updateConfigFromParameters();

    uint8_t hardwareStatus;

    // Use Position Control Mode
    write1Byte(ADDR_OPERATING_MODE, 4, &hardwareStatus);

    // Enable torque
    write1Byte(ADDR_TORQUE_ENABLE, 1, &hardwareStatus);
}

auto Servo::setPosition(ServoPosition position, ServoMode mode) -> Servo::ServoStatus
{
    // Convert degrees to ticks (0.0 - 360.0) to (0 to 4096)
    uint16_t targetPosition = static_cast<uint16_t>((position / 360.0f) * 4096.0f) % 4096;

    uint8_t hardwareStatus;

    uint32_t presentPosition;
    read4Byte(ADDR_PRESENT_POSITION, presentPosition, &hardwareStatus);
    
    // Get current position (make sure its positive)
    uint16_t currentPosition = static_cast<uint16_t>(((presentPosition % 4096) + 4096) % 4096);

    // Calculate the signed difference (accounting for overflow)
    int normalizedDifference = static_cast<int>(targetPosition - currentPosition);

    mAtLimit = false;

    switch (mode)
    {
        case ServoMode::Optimal:
            if (normalizedDifference > (SERVO_TICKS / 2)) {
                normalizedDifference -= SERVO_TICKS;  // Go the other (shorter) way around
            } else if (normalizedDifference < -(SERVO_TICKS / 2)) {
                normalizedDifference += SERVO_TICKS;  // Go the other (shorter) way around
            }
            break;
        case ServoMode::Clockwise: // clockwise
            if (normalizedDifference < 0)
            {
                normalizedDifference += SERVO_TICKS;
            }
            break;
        case ServoMode::CounterClockwise: // counter clockwise
            if (normalizedDifference > 0)
            {
                normalizedDifference -= SERVO_TICKS;
            }
            break;
        case ServoMode::Limited:
        {
            // Gets middle point between limits
            int middleLimit = (mForwardLimit + mReverseLimit) / 2;
            // Corrects middle limit if on wrong side
            if (mForwardLimit > mReverseLimit)
            {
                middleLimit += (SERVO_TICKS / 2);
            }
            middleLimit %= SERVO_TICKS;

            normalizedDifference = (targetPosition - currentPosition);
            // If the current path to the final position goes over the middle limit, go the other way
            if (middleLimit > currentPosition && middleLimit < targetPosition)
            {
                if (normalizedDifference > 0) normalizedDifference -= SERVO_TICKS;
                else if (normalizedDifference < 0) normalizedDifference += SERVO_TICKS;
            }

            mAtLimit = false;

            // Limit destination if between mForwardLimit and middleLimit
            if (upper(targetPosition) > mForwardLimit && targetPosition < upper(middleLimit)) {
                normalizedDifference = (mForwardLimit - currentPosition) % SERVO_TICKS;
                if (normalizedDifference < 0 && !(upper(currentPosition) > mForwardLimit && currentPosition < upper(middleLimit))) normalizedDifference += SERVO_TICKS;
                mAtLimit = true;
            }

            // Limit destination if between mReverseLimit and middleLimit
            else if (upper(targetPosition) > middleLimit && targetPosition < upper(mReverseLimit))
            {
                normalizedDifference = (mReverseLimit - currentPosition) % SERVO_TICKS;
                if (normalizedDifference > 0 && !(upper(currentPosition) > middleLimit && currentPosition < upper(mReverseLimit))) normalizedDifference -= SERVO_TICKS;
                mAtLimit = true;
            }
        }
    }

    // Calculate final goal position
    mGoalPosition = currentPosition + normalizedDifference;
    // Write goal position
    return write4Byte(ADDR_GOAL_POSITION, mGoalPosition, &hardwareStatus);
}

auto Servo::getTargetStatus() -> Servo::ServoStatus
{
    uint8_t hardwareStatus;
    uint32_t presentPosition;
    
    ServoStatus status = read4Byte(ADDR_PRESENT_POSITION, presentPosition, &hardwareStatus);
    if (status != ServoStatus::Success) return status;

    uint32_t mGoalPositionMod = mGoalPosition % SERVO_TICKS;

    if (presentPosition > mGoalPositionMod - SERVO_POSITION_DEAD_ZONE && presentPosition < mGoalPositionMod + SERVO_POSITION_DEAD_ZONE)
    {
        return ServoStatus::Success;
    }

    if (hardwareStatus != 0)
    {
        return ServoStatus::HardwareFailure;
    }

    return ServoStatus::Active;
}

auto Servo::getPosition(ServoPosition& position) -> Servo::ServoStatus
{
    uint8_t hardwareStatus;
    uint32_t positionInt;
    ServoStatus status = read4Byte(ADDR_PRESENT_POSITION, positionInt, &hardwareStatus);
    position = (static_cast<float>(positionInt % 4096) / 4096.0f) * 360.0f;
    return status;
}

auto Servo::getVelocity(ServoVelocity& velocity) -> Servo::ServoStatus
{
    uint8_t hardwareStatus;
    uint32_t velocity_int;
    Servo::ServoStatus status = read4Byte(ADDR_PRESENT_VELOCITY, velocity_int, &hardwareStatus);
    velocity = (static_cast<float>(static_cast<int32_t>(velocity_int)) * 0.22888f); // 0.22888f Conversion factor to get rot/sec (found in dynamixel wizard) 
    return status;
}

auto Servo::getCurrent(ServoCurrent& current) -> Servo::ServoStatus
{
    uint8_t hardwareStatus;
    uint16_t currentInt;
    ServoStatus status = read2Byte(ADDR_PRESENT_CURRENT, currentInt, &hardwareStatus);
    current = static_cast<float>(static_cast<int16_t>(currentInt))/ 1000.0f;
    return status;
}

auto Servo::getPositionAbsolute(ServoPosition& position) -> Servo::ServoStatus
{
    uint8_t hardwareStatus;
    uint32_t positionInt;
    ServoStatus status = read4Byte(ADDR_PRESENT_POSITION, positionInt, &hardwareStatus);
    position = (static_cast<float>(positionInt) / 4096.0f) * 360.0f;
    return status;
}

auto Servo::setProperty(ServoProperty prop, uint16_t value) -> Servo::ServoStatus
{
    uint8_t hardwareStatus;
    return write2Byte(static_cast<ServoAddr>(prop), value, &hardwareStatus);
}

auto Servo::write1Byte(ServoAddr addr, uint8_t data, uint8_t* hardwareStatus) const -> Servo::ServoStatus
{
    return static_cast<ServoStatus>(packetHandler->write1ByteTxRx(
        portHandler,
        mServoId,
        addr,
        data,
        hardwareStatus
    ));
}

auto Servo::write2Byte(ServoAddr addr, uint16_t data, uint8_t* hardwareStatus) const -> Servo::ServoStatus
{
    return static_cast<ServoStatus>(packetHandler->write2ByteTxRx(
        portHandler,
        mServoId,
        addr,
        data,
        hardwareStatus
    ));
}

auto Servo::write4Byte(ServoAddr addr, uint32_t data, uint8_t* hardwareStatus) const -> Servo::ServoStatus
{
    return static_cast<ServoStatus>(packetHandler->write4ByteTxRx(
        portHandler,
        mServoId,
        addr,
        data,
        hardwareStatus
    ));
}


auto Servo::read1Byte(ServoAddr addr, uint8_t& data, uint8_t* hardwareStatus) const -> Servo::ServoStatus
{
    return static_cast<ServoStatus>(packetHandler->read1ByteTxRx(
        portHandler,
        mServoId,
        addr,
        &data,
        hardwareStatus
    ));
}

auto Servo::read2Byte(ServoAddr addr, uint16_t& data, uint8_t* hardwareStatus) const -> Servo::ServoStatus
{
    return static_cast<ServoStatus>(packetHandler->read2ByteTxRx(
        portHandler,
        mServoId,
        addr,
        &data,
        hardwareStatus
    ));
}

auto Servo::read4Byte(ServoAddr addr, uint32_t& data, uint8_t* hardwareStatus) const -> Servo::ServoStatus
{
    return static_cast<ServoStatus>(packetHandler->read4ByteTxRx(
        portHandler,
        mServoId,
        addr,
        &data,
        hardwareStatus
    ));
}

auto Servo::init(const std::string& deviceName) -> Servo::ServoStatus
{
    portHandler = dynamixel::PortHandler::getPortHandler(deviceName.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int dxlCommResult;

    // Open Serial Port
    dxlCommResult = portHandler->openPort();
    if (!dxlCommResult) {
        return ServoStatus::FailedToOpenPort;
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    dxlCommResult = portHandler->setBaudRate(SERVO_BAUDRATE);
    if (!dxlCommResult) {
        return ServoStatus::FailedToSetBaud;
    }

    return ServoStatus::Success;
}

auto Servo::getLimitStatus() const -> bool
{
    return mAtLimit;
}
