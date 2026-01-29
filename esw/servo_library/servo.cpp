#include "servo.hpp"
#include "parameter.hpp"
#include <cstdint>

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.
#define SERVO_BAUDRATE 57600 // Baud rate of dynamixel devices

// Servo Addresses
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_PRESENT_CURRENT 126
#define ADDR_CURRENT_LIMIT 102
#define ADDR_VELOCITY_I_GAIN 76
#define ADDR_VELOCITY_P_GAIN 78
#define ADDR_POSITION_D_GAIN 80
#define ADDR_POSITION_I_GAIN 82
#define ADDR_POSITION_P_GAIN 84

#define SERVO_POSITION_DEAD_ZONE 5


using namespace mrover;

auto Servo::updateConfigFromParameters() -> void {
    float floatForwardLimit;
    float floatReverseLimit;
    std::vector<ParameterWrapper> parameters = {
            {std::format("{}.reverse_limit", mServoName), floatReverseLimit, 90.0f},
            {std::format("{}.forward_limit", mServoName), floatForwardLimit, 270.0f}
    };


    ParameterWrapper::declareParameters(mNode.get(), parameters);

    assert(forward_limit > 0.0f);
    assert(reverse_limit > 0.0f);
    assert(forward_limit < 360.0f);
    assert(reverse_limit < 360.0f);

    reverseLimit = static_cast<int>((reverseLimit / 360.0f) * 4096.0f);
    forwardLimit = static_cast<int>((forwardLimit / 360.0f) * 4096.0f);
}

Servo::Servo(rclcpp::Node::SharedPtr node, ServoId mServoId, const std::string& mServoName, ServoProperties properties) : mNode{std::move(node)}, mServoId{mServoId}, mServoName{mServoName}
{
  // ---------------------------------------------- Set servo properties ------------------------------------------------------ //
  // Position Gain
  setProperty(ServoProperty::PositionPGain, properties.positionPGain);
  setProperty(ServoProperty::PositionIGain, properties.positionIGain);
  setProperty(ServoProperty::PositionDGain, properties.positionDGain);

  // Velocity Gain
  setProperty(ServoProperty::VelocityPGain, properties.velocityPGain);
  setProperty(ServoProperty::VelocityIGain, properties.velocityIGain);

  // Current Limit
  setProperty(ServoProperty::CurrentLimit, properties.currentLimit);

  servoSetup();
}

Servo::Servo(rclcpp::Node::SharedPtr node, ServoId mServoId, const std::string& mServoName) : mNode{std::move(node)}, mServoId{mServoId}, mServoName{mServoName}
{
  servoSetup();
}

Servo::ServoStatus Servo::servoSetup()
{
    updateConfigFromParameters();
  forwardLimit = 270;
  reverseLimit = 90;

  uint8_t hardwareStatus;

  // Use Position Control Mode
  write1Byte(ADDR_OPERATING_MODE, 4, &hardwareStatus);

  // Enable torque
  write1Byte(ADDR_TORQUE_ENABLE, 1, &hardwareStatus);

  return Servo::ServoStatus::Success;
}

Servo::ServoStatus Servo::setPosition(ServoPosition position, ServoMode mode)
{

  uint32_t targetPosition = static_cast<uint32_t>((position / 360.0f) * 4096.0f) % 4096;

  uint8_t hardwareStatus;

  uint32_t presentPosition;
  read4Byte(ADDR_PRESENT_POSITION, presentPosition, &hardwareStatus);

  uint32_t currentPosition = presentPosition % 4096;

  // Calculate the signed difference (accounting for overflow)
  int32_t normalizedDifference = static_cast<int32_t>(targetPosition - currentPosition);

  switch (mode)
  {
    case ServoMode::Optimal:
      if (normalizedDifference > 2048) {
        normalizedDifference -= 4096;  // Go the other (shorter) way around
      } else if (normalizedDifference < -2048) {
        normalizedDifference += 4096;  // Go the other (shorter) way around
      }
      break;
    case ServoMode::Clockwise: // clockwise
      if (normalizedDifference < 0)
      {
        normalizedDifference += 4096;
      }
      break;
    case ServoMode::CounterClockwise: // counter clockwise
      if (normalizedDifference > 0)
      {
        normalizedDifference -= 4096;
      }
      break;
  }

  if (normalizedDifference > 0 && ((presentPosition + normalizedDifference) % 4096) > forwardLimit)
  {
    atLimit = true;
    int toLimit = ((presentPosition + normalizedDifference) % 4096) - forwardLimit;
    goalPosition = presentPosition + toLimit;
  }
  else if (normalizedDifference < 0 && ((presentPosition + normalizedDifference) % 4096) < reverseLimit)
  {
    atLimit = true;
    int toLimit = ((presentPosition + normalizedDifference) % 4096) + reverseLimit;
    goalPosition = presentPosition + toLimit;
  }
  else
  {
    atLimit = false;
    goalPosition = presentPosition + normalizedDifference;
  }

  // Calculate the optimal goal position

  // Write goal position
  return write4Byte(ADDR_GOAL_POSITION, goalPosition, &hardwareStatus);
}

Servo::ServoStatus Servo::getTargetStatus()
{
  uint8_t hardwareStatus;
  uint32_t presentPosition;
  read4Byte(ADDR_PRESENT_POSITION, presentPosition, &hardwareStatus);

  uint32_t goalPositionMod = goalPosition % 4096;

  if (presentPosition > goalPositionMod - SERVO_POSITION_DEAD_ZONE && presentPosition < goalPositionMod + SERVO_POSITION_DEAD_ZONE)
  {
    return ServoStatus::Success;
  }

  if (hardwareStatus != 0)
  {
    return ServoStatus::HardwareFailure;
  }

  return ServoStatus::Active;
}

Servo::ServoStatus Servo::getPosition(ServoPosition& position)
{
  uint8_t hardwareStatus;
  uint32_t positionInt;
  ServoStatus status = read4Byte(ADDR_PRESENT_POSITION, positionInt, &hardwareStatus);
  position = (static_cast<float>(positionInt % 4096) / 4096.0f) * 360.0f;
  return status;
}

Servo::ServoStatus Servo::getVelocity(ServoVelocity& velocity)
{
  uint8_t hardwareStatus;
  uint32_t velocity_int;
  Servo::ServoStatus status = read4Byte(ADDR_PRESENT_VELOCITY, velocity_int, &hardwareStatus);
  velocity = (static_cast<float>(static_cast<int32_t>(velocity_int)) * 0.22888f);
  return status;
}

Servo::ServoStatus Servo::getCurrent(ServoCurrent& current)
{
  uint8_t hardwareStatus;
  uint16_t currentInt;
  ServoStatus status = read2Byte(ADDR_PRESENT_CURRENT, currentInt, &hardwareStatus);
  current = static_cast<float>(static_cast<int16_t>(currentInt))/ 1000.0f;
  return status;
}

Servo::ServoStatus Servo::getPositionAbsolute(ServoPosition& position)
{
  uint8_t hardwareStatus;
  uint32_t positionInt;
  ServoStatus status = read4Byte(ADDR_PRESENT_POSITION, positionInt, &hardwareStatus);
  position = (static_cast<float>(positionInt) / 4096.0f) * 360.0f;
  return status;
}

Servo::ServoStatus Servo::setProperty(ServoProperty prop, uint16_t value)
{
  uint8_t hardwareStatus;
  return write2Byte(static_cast<ServoAddr>(prop), value, &hardwareStatus);
}

Servo::ServoStatus Servo::write1Byte(ServoAddr addr, uint8_t data, uint8_t* hardwareStatus) const
{
  return static_cast<ServoStatus>(packetHandler->write1ByteTxRx(
    portHandler,
    mServoId,
    addr,
    data,
    hardwareStatus
  ));
}

Servo::ServoStatus Servo::write2Byte(ServoAddr addr, uint16_t data, uint8_t* hardwareStatus) const
{
  return static_cast<ServoStatus>(packetHandler->write2ByteTxRx(
    portHandler,
    mServoId,
    addr,
    data,
    hardwareStatus
  ));
}

Servo::ServoStatus Servo::write4Byte(ServoAddr addr, uint32_t data, uint8_t* hardwareStatus) const
{
  return static_cast<ServoStatus>(packetHandler->write4ByteTxRx(
    portHandler,
    mServoId,
    addr,
    data,
    hardwareStatus
  ));
}


Servo::ServoStatus Servo::read1Byte(ServoAddr addr, uint8_t& data, uint8_t* hardwareStatus) const
{
  return static_cast<ServoStatus>(packetHandler->read1ByteTxRx(
    portHandler,
    mServoId,
    addr,
    &data,
    hardwareStatus
  ));
}

Servo::ServoStatus Servo::read2Byte(ServoAddr addr, uint16_t& data, uint8_t* hardwareStatus) const
{
  return static_cast<ServoStatus>(packetHandler->read2ByteTxRx(
    portHandler,
    mServoId,
    addr,
    &data,
    hardwareStatus
  ));
}

Servo::ServoStatus Servo::read4Byte(ServoAddr addr, uint32_t& data, uint8_t* hardwareStatus) const
{
  return static_cast<ServoStatus>(packetHandler->read4ByteTxRx(
    portHandler,
    mServoId,
    addr,
    &data,
    hardwareStatus
  ));
}

Servo::ServoStatus Servo::init(const std::string& deviceName)
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
}

bool Servo::getLimitStatus()
{
  return atLimit;
}
