#include "servo.hpp"
#include <cstdint>

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.
#define SERVO_BAUDRATE 57600 // Baud rate of dynamixel devices

// Servo Addresses
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_CURRENT 126
#define ADDR_CURRENT_LIMIT 102
#define ADDR_VELOCITY_I_GAIN 76
#define ADDR_VELOCITY_P_GAIN 78
#define ADDR_POSITION_D_GAIN 80
#define ADDR_POSITION_I_GAIN 82
#define ADDR_POSITION_P_GAIN 84

using namespace mrover;

Servo::Servo(ServoId id, ServoProperties properties) : id{id}
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

Servo::Servo(ServoId id) : id{id}
{
  servoSetup();
}

Servo::ServoStatus Servo::servoSetup()
{
  uint8_t hardwareStatus;

  // Use Position Control Mode
  write1Byte(ADDR_OPERATING_MODE, 4, &hardwareStatus);

  // Enable torque
  write1Byte(ADDR_TORQUE_ENABLE, 1, &hardwareStatus);

  return Servo::ServoStatus::Success;
}

Servo::ServoStatus Servo::setPosition(ServoPosition position, ServoMode mode)
{
  uint32_t targetPosition = position % 4096;

  uint8_t hardwareStatus;

  uint32_t presentPosition;
  read4Byte(ADDR_PRESENT_POSITION, presentPosition, &hardwareStatus);

  uint32_t goalPosition;
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

  // Calculate the optimal goal position
  goalPosition = presentPosition + normalizedDifference;

  // Write goal position
  return write4Byte(ADDR_GOAL_POSITION, goalPosition, &hardwareStatus);
}

Servo::ServoStatus Servo::getPosition(ServoPosition& position)
{
  Servo::ServoStatus status = getPositionAbsolute(position);
  position %= 4096;
  return status;
}

Servo::ServoStatus Servo::getPositionAbsolute(ServoPosition& position)
{
  uint8_t hardwareStatus;
  return read4Byte(ADDR_PRESENT_POSITION, position, &hardwareStatus);
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
    id,
    addr,
    data,
    hardwareStatus
  ));
}

Servo::ServoStatus Servo::write2Byte(ServoAddr addr, uint16_t data, uint8_t* hardwareStatus) const
{
  return static_cast<ServoStatus>(packetHandler->write2ByteTxRx(
    portHandler,
    id,
    addr,
    data,
    hardwareStatus
  ));
}

Servo::ServoStatus Servo::write4Byte(ServoAddr addr, uint32_t data, uint8_t* hardwareStatus) const
{
  return static_cast<ServoStatus>(packetHandler->write4ByteTxRx(
    portHandler,
    id,
    addr,
    data,
    hardwareStatus
  ));
}


Servo::ServoStatus Servo::read1Byte(ServoAddr addr, uint8_t& data, uint8_t* hardwareStatus) const
{
  return static_cast<ServoStatus>(packetHandler->read1ByteTxRx(
    portHandler,
    id,
    addr,
    &data,
    hardwareStatus
  ));
}

Servo::ServoStatus Servo::read2Byte(ServoAddr addr, uint16_t& data, uint8_t* hardwareStatus) const
{
  return static_cast<ServoStatus>(packetHandler->read2ByteTxRx(
    portHandler,
    id,
    addr,
    &data,
    hardwareStatus
  ));
}

Servo::ServoStatus Servo::read4Byte(ServoAddr addr, uint32_t& data, uint8_t* hardwareStatus) const
{
  return static_cast<ServoStatus>(packetHandler->read4ByteTxRx(
    portHandler,
    id,
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