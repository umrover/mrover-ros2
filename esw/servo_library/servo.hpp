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

  Servo(rclcpp::Node::SharedPtr node, ServoId id, std::string name);

  auto setPosition(ServoPosition position, ServoMode mode) -> ServoStatus;
  auto getPosition(ServoPosition& position) -> ServoStatus;
  auto getVelocity(ServoVelocity& velocity) -> ServoStatus;
  auto getCurrent(ServoCurrent& current) -> ServoStatus;
  auto getPositionAbsolute(ServoPosition& position) -> ServoStatus;
  auto setProperty(ServoProperty prop, uint16_t value) -> ServoStatus;
  auto getTargetStatus() -> ServoStatus;
  [[nodiscard]] auto getLimitStatus() const -> bool;

  static auto init(const std::string& deviceName) -> ServoStatus;

private:
  auto updateConfigFromParameters() -> void;

  inline auto write1Byte(ServoAddr addr, uint8_t data, uint8_t* hardwareStatus) const -> Servo::ServoStatus;
  inline auto write2Byte(ServoAddr addr, uint16_t data, uint8_t* hardwareStatus) const -> Servo::ServoStatus;
  inline auto write4Byte(ServoAddr addr, uint32_t data, uint8_t* hardwareStatus) const -> Servo::ServoStatus;

  inline auto read1Byte(ServoAddr addr, uint8_t& data, uint8_t* hardwareStatus) const -> Servo::ServoStatus;
  inline auto read2Byte(ServoAddr addr, uint16_t& data, uint8_t* hardwareStatus) const -> Servo::ServoStatus;
  inline auto read4Byte(ServoAddr addr, uint32_t& data, uint8_t* hardwareStatus) const -> Servo::ServoStatus;

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