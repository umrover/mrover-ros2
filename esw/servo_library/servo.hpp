#pragma once


#include <cstdint>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_sdk/port_handler.h>
#include <dynamixel_sdk/packet_handler.h>
#include <parameter.hpp>
#include <string>

#define DEFAULT_CURRENT_LIMIT 1750
#define DEFAULT_POSITION_P_GAIN 400
#define DEFAULT_POSITION_I_GAIN 0
#define DEFAULT_POSITION_D_GAIN 0
#define DEFAULT_VELOCITY_P_GAIN 180
#define DEFAULT_VELOCITY_I_GAIN 0

namespace mrover {

class Servo
{
  using ServoId = uint8_t;
  using ServoPosition = float;
  using ServoVelocity = float;
  using ServoCurrent = float;
  using ServoAddr = uint16_t;

public:

  enum class ServoStatus
  {
    Active = 400,
    HardwareFailure = 401,
    Success = 0,
    FailedToOpenPort,
    FailedToSetBaud,
    CommPortBusy = -1000,
    CommTxFail = -1001,
    CommRxFail = -1002,
    CommTxError = -2000,
    CommRxWaiting = -3000,
    CommRxTimeout = -3001,
    CommRxCorrupt = -3002,
    CommNotAvailable = -9000,
  };

  enum class ServoProperty
  {
    PositionPGain = 84,
    PositionIGain = 82,
    PositionDGain = 80,
    VelocityPGain = 78,
    VelocityIGain = 76,
    CurrentLimit = 102,
  };

  enum class ServoMode
  {
    Optimal,
    Clockwise,
    CounterClockwise,
    Limited,
  };

  struct ServoProperties
  {
    uint16_t positionPGain = DEFAULT_POSITION_P_GAIN;
    uint16_t positionIGain = DEFAULT_POSITION_I_GAIN;
    uint16_t positionDGain = DEFAULT_POSITION_D_GAIN;
    uint16_t velocityPGain = DEFAULT_VELOCITY_P_GAIN;
    uint16_t velocityIGain = DEFAULT_VELOCITY_I_GAIN;
    uint16_t currentLimit = DEFAULT_CURRENT_LIMIT;

    ServoProperties(const ServoProperties& state) = default;
    ServoProperties() = default;
  };
bool moveToTarget(
    int startPos,          // [0, 4096]
    int forwardLimit,      // >= 0
    int reverseLimit,      // >= 0
    int targetPos,         // desired position
    int* distanceOut       // signed distance traveled
);
  Servo(rclcpp::Node::SharedPtr node, ServoId id, const std::string& name, ServoProperties properties);
  Servo(rclcpp::Node::SharedPtr node, ServoId id, const std::string& name);

  ServoStatus setPosition(ServoPosition position, ServoMode mode);
  ServoStatus getPosition(ServoPosition& position);
  ServoStatus getVelocity(ServoVelocity& velocity);
  ServoStatus getCurrent(ServoCurrent& current);
  ServoStatus getPositionAbsolute(ServoPosition& position);
  ServoStatus setProperty(ServoProperty prop, uint16_t value);
  ServoStatus getTargetStatus();
  bool getLimitStatus() const;

  static ServoStatus init(const std::string& deviceName);

private:
  auto updateConfigFromParameters() -> void;

  ServoStatus servoSetup();

  inline ServoStatus write1Byte(ServoAddr addr, uint8_t data, uint8_t* hardwareStatus) const;
  inline ServoStatus write2Byte(ServoAddr addr, uint16_t data, uint8_t* hardwareStatus) const;
  inline ServoStatus write4Byte(ServoAddr addr, uint32_t data, uint8_t* hardwareStatus) const;

  inline ServoStatus read1Byte(ServoAddr addr, uint8_t& data, uint8_t* hardwareStatus) const;
  inline ServoStatus read2Byte(ServoAddr addr, uint16_t& data, uint8_t* hardwareStatus) const;
  inline ServoStatus read4Byte(ServoAddr addr, uint32_t& data, uint8_t* hardwareStatus) const;

  inline static dynamixel::PortHandler *portHandler;
  inline static dynamixel::PacketHandler *packetHandler;

  ServoId mServoId;
  std::string mServoName;
  int forwardLimit;
  int reverseLimit;
  uint32_t goalPosition;

  bool atLimit = false;

  rclcpp::Node::SharedPtr mNode;
};

}